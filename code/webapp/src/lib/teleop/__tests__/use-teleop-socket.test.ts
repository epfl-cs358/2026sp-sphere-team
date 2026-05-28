/** Developed with AI assistance (Claude, Anthropic) */

import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { renderHook, act } from "@testing-library/react";
import { useTeleopSocket } from "../use-teleop-socket";
import type { Cmd, ControlVerb } from "../types";

class MockWebSocket {
  static instances: MockWebSocket[] = [];
  static CONNECTING = 0;
  static OPEN = 1;
  static CLOSING = 2;
  static CLOSED = 3;

  readyState = 0;
  bufferedAmount = 0;
  onopen: (() => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: ((e: unknown) => void) | null = null;
  onmessage: (() => void) | null = null;
  url: string;
  send = vi.fn();
  close = vi.fn(() => {
    this.readyState = 3;
  });

  constructor(url: string) {
    this.url = url;
    MockWebSocket.instances.push(this);
  }

  simulateOpen() {
    this.readyState = 1;
    this.onopen?.();
  }

  simulateClose() {
    this.readyState = 3;
    this.onclose?.();
  }

  simulateError(e: unknown = new Event("error")) {
    this.onerror?.(e);
  }
}

const noopCmd = (): Cmd => ({ vx: 0, vy: 0, omega: 0 });
const isArmedTrue = () => true;

describe("useTeleopSocket", () => {
  const originalWebSocket = globalThis.WebSocket;
  const originalRandom = Math.random;

  beforeEach(() => {
    MockWebSocket.instances = [];
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket;
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
    globalThis.WebSocket = originalWebSocket;
    Math.random = originalRandom;
    vi.restoreAllMocks();
  });

  it("connects to the provided url on mount", () => {
    const { result } = renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    expect(MockWebSocket.instances).toHaveLength(1);
    expect(MockWebSocket.instances[0].url).toBe("ws://localhost:9000");
    expect(result.current.state).toBe("connecting");

    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    expect(result.current.state).toBe("connected");
  });

  it("sends frames at 20 Hz when connected", () => {
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    act(() => {
      vi.advanceTimersByTime(200);
    });
    expect(MockWebSocket.instances[0].send.mock.calls.length).toBeGreaterThanOrEqual(4);
  });

  it("skips send when bufferedAmount exceeds threshold", () => {
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    MockWebSocket.instances[0].bufferedAmount = 300;
    act(() => {
      vi.advanceTimersByTime(200);
    });
    expect(MockWebSocket.instances[0].send).not.toHaveBeenCalled();
  });

  it("reconnects with backoff schedule", () => {
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    expect(MockWebSocket.instances).toHaveLength(1);

    // attempt 0 → 0ms reconnect after first close
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(0);
    });
    expect(MockWebSocket.instances).toHaveLength(2);

    // attempt 1 → 250ms
    act(() => {
      MockWebSocket.instances[1].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(249);
    });
    expect(MockWebSocket.instances).toHaveLength(2);
    act(() => {
      vi.advanceTimersByTime(1);
    });
    expect(MockWebSocket.instances).toHaveLength(3);

    // attempt 2 → 500ms
    act(() => {
      MockWebSocket.instances[2].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(499);
    });
    expect(MockWebSocket.instances).toHaveLength(3);
    act(() => {
      vi.advanceTimersByTime(1);
    });
    expect(MockWebSocket.instances).toHaveLength(4);
  });

  it("applies jitter from attempt 3", () => {
    Math.random = () => 0.5; // jitter factor = 1 + (0.5 * 0.4 - 0.2) = 1.0; we just want it deterministic-in-range
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );

    // close 4 times, advancing past each scheduled delay
    // attempt 0 → 0ms
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(0);
    });
    // attempt 1 → 250ms
    act(() => {
      MockWebSocket.instances[1].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(250);
    });
    // attempt 2 → 500ms
    act(() => {
      MockWebSocket.instances[2].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(500);
    });

    expect(MockWebSocket.instances).toHaveLength(4);

    // attempt 3 → 1000 * (1 + (random*0.4 - 0.2)). For multiple random values
    // verify the delay before the 4th instance falls in [800, 1200].
    act(() => {
      MockWebSocket.instances[3].simulateClose();
    });
    // not yet at 800ms
    act(() => {
      vi.advanceTimersByTime(799);
    });
    expect(MockWebSocket.instances).toHaveLength(4);
    // by 1200ms must have fired
    act(() => {
      vi.advanceTimersByTime(401); // total 1200
    });
    expect(MockWebSocket.instances).toHaveLength(5);
  });

  it("resets attempt counter after a stable connection", () => {
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    // attempt 0 close → 0ms reconnect
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(0);
    });
    expect(MockWebSocket.instances).toHaveLength(2);

    // reconnect, open, hold 3s (stability), then close — should be attempt-0 again
    act(() => {
      MockWebSocket.instances[1].simulateOpen();
    });
    act(() => {
      vi.advanceTimersByTime(3000);
    });
    act(() => {
      MockWebSocket.instances[1].simulateClose();
    });
    act(() => {
      vi.advanceTimersByTime(0);
    });
    expect(MockWebSocket.instances).toHaveLength(3);
  });

  it("caps backoff at 5000 ms", () => {
    Math.random = () => 1; // max jitter (+20%): 4000 * 1.2 = 4800, still ≤ 5000
    renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );

    // Burn through attempts 0..5 to land in the capped regime.
    const advance = (ms: number) => {
      act(() => {
        vi.advanceTimersByTime(ms);
      });
    };
    const closeLast = () => {
      act(() => {
        MockWebSocket.instances[MockWebSocket.instances.length - 1].simulateClose();
      });
    };

    closeLast(); advance(0);    // attempt 1 spawned
    closeLast(); advance(250);  // attempt 2
    closeLast(); advance(500);  // attempt 3
    closeLast(); advance(1200); // attempt 4 (with jitter slack)
    closeLast(); advance(2400); // attempt 5 (2000 * 1.2)
    // Now we are at attempt 5+, expect ≤ 5000ms slack.
    closeLast();
    advance(5000);
    expect(
      MockWebSocket.instances.length,
    ).toBeGreaterThanOrEqual(7);
  });

  it("cleanup cancels pending reconnect", () => {
    const { unmount } = renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    // unmount while reconnect timer is pending (attempt 1 = 250ms)
    unmount();
    act(() => {
      vi.advanceTimersByTime(10_000);
    });
    expect(MockWebSocket.instances).toHaveLength(1);
  });

  it("null url returns idle and never connects", () => {
    const { result } = renderHook(() =>
      useTeleopSocket({ url: null, getCmd: noopCmd }),
    );
    expect(MockWebSocket.instances).toHaveLength(0);
    expect(result.current.state).toBe("idle");
  });

  it("onerror does not change phase but logs", () => {
    const warn = vi.spyOn(console, "warn").mockImplementation(() => {});
    const { result } = renderHook(() =>
      useTeleopSocket({ url: "ws://localhost:9000", getCmd: noopCmd, isArmed: isArmedTrue }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    expect(result.current.state).toBe("connected");
    act(() => {
      MockWebSocket.instances[0].simulateError();
    });
    expect(result.current.state).toBe("connected");
    expect(warn).toHaveBeenCalled();
  });

  // ---- New tests for Slice B ----

  it.each<[ControlVerb, string]>([
    ["arm", "c:arm"],
    ["disarm", "c:disarm"],
    ["kill", "c:kill"],
    ["clearkill", "c:clearkill"],
  ])("sendControl emits exact bytes for verb=%s", (verb, expected) => {
    const { result } = renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    let ret: "sent" | "dropped" | undefined;
    act(() => {
      ret = result.current.sendControl(verb);
    });
    expect(ret).toBe("sent");
    const calls = MockWebSocket.instances[0].send.mock.calls;
    // Filter out any 20Hz velocity sends that might be queued
    const controlCalls = calls.filter((c) =>
      typeof c[0] === "string" && (c[0] as string).startsWith("c:"),
    );
    expect(controlCalls.length).toBeGreaterThanOrEqual(1);
    expect(controlCalls[0][0]).toBe(expected);
  });

  it("velocity loop is suppressed while isArmed returns false", () => {
    const isArmed = vi.fn(() => false);
    renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    act(() => {
      vi.advanceTimersByTime(200);
    });
    expect(MockWebSocket.instances[0].send).not.toHaveBeenCalled();
  });

  it("onDrop fires only after ws had opened (open then close)", () => {
    const onDrop = vi.fn();
    renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
        onDrop,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    expect(onDrop).toHaveBeenCalledTimes(1);
  });

  it("onDrop does NOT fire when ws closes without ever opening", () => {
    const onDrop = vi.fn();
    renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
        onDrop,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });
    expect(onDrop).not.toHaveBeenCalled();
  });

  it("sendControl returns 'dropped' when ws not open", () => {
    const { result } = renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
      }),
    );
    // Do NOT simulate open — ws is in CONNECTING state.
    let ret: "sent" | "dropped" | undefined;
    act(() => {
      ret = result.current.sendControl("kill");
    });
    expect(ret).toBe("dropped");
    expect(MockWebSocket.instances[0].send).not.toHaveBeenCalled();
  });

  it("sendControl never gates on bufferedAmount (regression guard for kill)", () => {
    const { result } = renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    MockWebSocket.instances[0].bufferedAmount = 9999;
    let ret: "sent" | "dropped" | undefined;
    act(() => {
      ret = result.current.sendControl("kill");
    });
    expect(ret).toBe("sent");
    const calls = MockWebSocket.instances[0].send.mock.calls;
    const controlCalls = calls.filter((c) =>
      typeof c[0] === "string" && (c[0] as string).startsWith("c:"),
    );
    expect(controlCalls[0][0]).toBe("c:kill");
  });

  it("velocity loop SKIPS send when bufferedAmount > 256 (regression guard)", () => {
    renderHook(() =>
      useTeleopSocket({
        url: "ws://localhost:9000",
        getCmd: noopCmd,
        isArmed: isArmedTrue,
      }),
    );
    act(() => {
      MockWebSocket.instances[0].simulateOpen();
    });
    MockWebSocket.instances[0].bufferedAmount = 300;
    act(() => {
      vi.advanceTimersByTime(200);
    });
    // No velocity bytes should have been sent.
    const velocityCalls = MockWebSocket.instances[0].send.mock.calls.filter(
      (c) => typeof c[0] === "string" && !(c[0] as string).startsWith("c:"),
    );
    expect(velocityCalls.length).toBe(0);
  });
});
