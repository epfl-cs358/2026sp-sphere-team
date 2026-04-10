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
    source.connect("ws://localhost:3001");
    expect(MockWebSocket.instances).toHaveLength(1);
    expect(MockWebSocket.instances[0].url).toBe("ws://localhost:3001");
  });

  it("sets binaryType to arraybuffer", () => {
    source.connect("ws://localhost:3001");
    expect(MockWebSocket.instances[0].binaryType).toBe("arraybuffer");
  });

  it("sets connected to true on open", () => {
    source.connect("ws://localhost:3001");
    MockWebSocket.instances[0].simulateOpen();
    expect(source.connected).toBe(true);
  });

  it("sets connected to false on close", () => {
    source.connect("ws://localhost:3001");
    MockWebSocket.instances[0].simulateOpen();
    MockWebSocket.instances[0].close();
    expect(source.connected).toBe(false);
  });

  it("calls onError when WebSocket errors", () => {
    const onError = vi.fn();
    source.onError = onError;
    source.connect("ws://localhost:3001");
    MockWebSocket.instances[0].simulateError();
    expect(onError).toHaveBeenCalledWith(
      expect.objectContaining({ message: expect.stringContaining("ws://localhost:3001") })
    );
  });

  it("closes previous connection on reconnect", () => {
    source.connect("ws://localhost:3001");
    const first = MockWebSocket.instances[0];
    source.connect("ws://localhost:3002");
    expect(first.close).toHaveBeenCalled();
    expect(MockWebSocket.instances).toHaveLength(2);
  });

  it("disconnect closes WebSocket", () => {
    source.connect("ws://localhost:3001");
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

    source.connect("ws://localhost:3001");
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
    source.onFrame = onFrame;
    source.connect("ws://localhost:3001");
    const ws = MockWebSocket.instances[0];
    ws.simulateOpen();

    // Send string instead of ArrayBuffer
    ws.onmessage?.({ data: "not binary" as unknown as ArrayBuffer });

    expect(onFrame).not.toHaveBeenCalled();
  });
});
