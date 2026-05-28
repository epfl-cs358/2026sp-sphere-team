import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { WebSocketSource } from "../websocket-source";

class MockWebSocket {
  static instances: MockWebSocket[] = [];
  binaryType = "blob";
  readyState = 0; // CONNECTING
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: ArrayBuffer }) => void) | null = null;
  onerror: (() => void) | null = null;
  onclose: (() => void) | null = null;
  url: string;
  close = vi.fn(() => {
    this.readyState = 3;
    this.onclose?.();
  });

  constructor(url: string) {
    this.url = url;
    MockWebSocket.instances.push(this);
  }

  simulateOpen() {
    this.readyState = 1;
    this.onopen?.();
  }

  simulateError() {
    this.onerror?.();
  }

  simulateMessage(data: ArrayBuffer) {
    this.onmessage?.({ data });
  }
}

const noopCallbacks = { onFrame: vi.fn(), onError: vi.fn() };

describe("WebSocketSource", () => {
  let source: WebSocketSource;
  const originalWebSocket = globalThis.WebSocket;

  beforeEach(() => {
    MockWebSocket.instances = [];
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket;
    source = new WebSocketSource();
  });

  afterEach(() => {
    source.disconnect();
    globalThis.WebSocket = originalWebSocket;
  });

  it("has type 'websocket'", () => {
    expect(source.type).toBe("websocket");
  });

  it("starts disconnected", () => {
    expect(source.connected).toBe(false);
  });

  it("creates a WebSocket connection with the given URL", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    expect(MockWebSocket.instances).toHaveLength(1);
    expect(MockWebSocket.instances[0].url).toBe("ws://localhost:3001");
  });

  it("sets binaryType to arraybuffer", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    expect(MockWebSocket.instances[0].binaryType).toBe("arraybuffer");
  });

  it("sets connected to true on open", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    MockWebSocket.instances[0].simulateOpen();
    expect(source.connected).toBe(true);
  });

  it("sets connected to false on close", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    MockWebSocket.instances[0].simulateOpen();
    MockWebSocket.instances[0].close();
    expect(source.connected).toBe(false);
  });

  it("calls onError when WebSocket errors", () => {
    const onError = vi.fn();
    source.connect("ws://localhost:3001", { onFrame: vi.fn(), onError });
    MockWebSocket.instances[0].simulateError();
    expect(onError).toHaveBeenCalledWith(
      expect.objectContaining({ message: expect.stringContaining("ws://localhost:3001") })
    );
  });

  it("closes previous connection on reconnect", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    const first = MockWebSocket.instances[0];
    source.connect("ws://localhost:3002", noopCallbacks);
    expect(first.close).toHaveBeenCalled();
    expect(MockWebSocket.instances).toHaveLength(2);
  });

  it("disconnect closes WebSocket", () => {
    source.connect("ws://localhost:3001", noopCallbacks);
    const ws = MockWebSocket.instances[0];
    source.disconnect();
    expect(ws.close).toHaveBeenCalled();
    expect(source.connected).toBe(false);
  });

  it("creates an Image from binary message data", () => {
    const origURL = globalThis.URL;
    const mockObjectUrl = "blob:mock-url";
    globalThis.URL.createObjectURL = vi.fn(() => mockObjectUrl);
    globalThis.URL.revokeObjectURL = vi.fn();

    const images: HTMLImageElement[] = [];
    const origImage = globalThis.Image;
    globalThis.Image = class extends origImage {
      constructor() {
        super();
        images.push(this);
      }
    } as typeof Image;

    source.connect("ws://localhost:3001", noopCallbacks);
    const ws = MockWebSocket.instances[0];
    ws.simulateOpen();

    const jpegData = new Uint8Array([0xff, 0xd8, 0xff, 0xe0]).buffer;
    ws.simulateMessage(jpegData);

    expect(URL.createObjectURL).toHaveBeenCalled();
    expect(images.length).toBeGreaterThan(0);
    expect(images[0].src).toContain(mockObjectUrl);

    globalThis.Image = origImage;
    globalThis.URL = origURL;
  });

  it("ignores non-ArrayBuffer messages", () => {
    const onFrame = vi.fn();
    source.connect("ws://localhost:3001", { onFrame, onError: vi.fn() });
    const ws = MockWebSocket.instances[0];
    ws.simulateOpen();

    // Send string instead of ArrayBuffer
    ws.onmessage?.({ data: "not binary" as unknown as ArrayBuffer });

    expect(onFrame).not.toHaveBeenCalled();
  });

  describe("reconnect behaviour", () => {
    beforeEach(() => {
      vi.useFakeTimers();
    });

    afterEach(() => {
      vi.useRealTimers();
    });

    it("reconnects after onclose using the backoff schedule", () => {
      source.connect("ws://localhost:3001", noopCallbacks);
      expect(MockWebSocket.instances).toHaveLength(1);

      // attempt 0 → 0 ms
      MockWebSocket.instances[0].close();
      vi.advanceTimersByTime(0);
      expect(MockWebSocket.instances).toHaveLength(2);

      // attempt 1 → 250 ms
      MockWebSocket.instances[1].close();
      vi.advanceTimersByTime(249);
      expect(MockWebSocket.instances).toHaveLength(2);
      vi.advanceTimersByTime(1);
      expect(MockWebSocket.instances).toHaveLength(3);

      // attempt 2 → 500 ms
      MockWebSocket.instances[2].close();
      vi.advanceTimersByTime(499);
      expect(MockWebSocket.instances).toHaveLength(3);
      vi.advanceTimersByTime(1);
      expect(MockWebSocket.instances).toHaveLength(4);
    });

    it("applies jitter from attempt 3", () => {
      // Pin Math.random so jitter is deterministic but non-zero.
      const randSpy = vi.spyOn(Math, "random").mockReturnValue(0.5);

      source.connect("ws://localhost:3001", noopCallbacks);
      // attempts 0,1,2 burn through without jitter; advance generously.
      for (let i = 0; i < 3; i++) {
        MockWebSocket.instances[i].close();
        vi.advanceTimersByTime(1000);
      }
      // Now the 4th open is the one closed → attempt 3 reconnect with jitter.
      const before = MockWebSocket.instances.length;
      MockWebSocket.instances[before - 1].close();

      // Should be in [800, 1200] ms (1000 ± 20 %).
      vi.advanceTimersByTime(799);
      expect(MockWebSocket.instances).toHaveLength(before);
      vi.advanceTimersByTime(401); // total 1200
      expect(MockWebSocket.instances.length).toBeGreaterThan(before);

      randSpy.mockRestore();
    });

    it("resets attempt counter after a stable connection", () => {
      source.connect("ws://localhost:3001", noopCallbacks);
      MockWebSocket.instances[0].simulateOpen();
      // Stable for 3 s → counter resets.
      vi.advanceTimersByTime(3000);
      MockWebSocket.instances[0].close();
      // Next reconnect should be attempt 0 → ~0 ms.
      vi.advanceTimersByTime(0);
      expect(MockWebSocket.instances).toHaveLength(2);
    });

    it("caps backoff at 5000 ms", () => {
      // With Math.random=1, jitter factor is at max (+20 %): 4000 * 1.2 = 4800
      // which is below the 5000 cap. We just assert no delay exceeds the cap.
      const randSpy = vi.spyOn(Math, "random").mockReturnValue(0.999);
      source.connect("ws://localhost:3001", noopCallbacks);
      for (let i = 0; i < 10; i++) {
        const ws = MockWebSocket.instances[MockWebSocket.instances.length - 1];
        ws.close();
        // 5000 ms is the cap; advancing this much must always produce the next instance.
        vi.advanceTimersByTime(5000);
        expect(MockWebSocket.instances.length).toBe(i + 2);
      }
      randSpy.mockRestore();
    });

    it("disconnect() cancels a pending reconnect", () => {
      source.connect("ws://localhost:3001", noopCallbacks);
      MockWebSocket.instances[0].simulateOpen();
      MockWebSocket.instances[0].close();
      // Reconnect timer is now pending (attempt 1 = 250 ms).
      source.disconnect();
      vi.advanceTimersByTime(10_000);
      expect(MockWebSocket.instances).toHaveLength(1);
    });

    it("disconnect() during open state cancels future reconnects", () => {
      source.connect("ws://localhost:3001", noopCallbacks);
      MockWebSocket.instances[0].simulateOpen();
      source.disconnect();
      // The disconnect-driven close should NOT enqueue a reconnect.
      vi.advanceTimersByTime(10_000);
      expect(MockWebSocket.instances).toHaveLength(1);
    });

    it("connect(newUrl) cancels any pending reconnect from the previous url", () => {
      source.connect("ws://localhost:3001", noopCallbacks);
      MockWebSocket.instances[0].close();
      // attempt 0 fires at 0 ms, so before any advance the timer is pending.
      // Re-connect to a different url before letting the timer run.
      source.connect("ws://localhost:3002", noopCallbacks);
      // Two instances exist (the original + the explicit reconnect).
      expect(MockWebSocket.instances).toHaveLength(2);
      // No phantom 3rd instance should appear from the old pending timer.
      vi.advanceTimersByTime(10_000);
      expect(MockWebSocket.instances).toHaveLength(2);
      expect(MockWebSocket.instances[1].url).toBe("ws://localhost:3002");
    });

    it("onerror still propagates to onError", () => {
      const onError = vi.fn();
      source.connect("ws://localhost:3001", { onFrame: vi.fn(), onError });
      MockWebSocket.instances[0].simulateError();
      expect(onError).toHaveBeenCalledTimes(1);
      expect(onError.mock.calls[0][0]).toBeInstanceOf(Error);
    });
  });
});
