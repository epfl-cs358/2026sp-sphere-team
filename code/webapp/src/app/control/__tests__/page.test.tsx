/** Developed with AI assistance (Claude, Anthropic) */

import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { render, screen, act, fireEvent, cleanup } from "@testing-library/react";
import ControlPage from "../page";

// Shared MockWebSocket — re-used by every test. Reset in beforeEach.
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
}

function connectAndOpen() {
  // The page boots disconnected. Click "Connect" to enable, then simulate open.
  const connect = screen.getByRole("button", { name: /^connect$/i });
  fireEvent.click(connect);
  act(() => {
    MockWebSocket.instances[0].simulateOpen();
  });
}

function lastWs(): MockWebSocket {
  return MockWebSocket.instances[MockWebSocket.instances.length - 1];
}

describe("ControlPage arm/disarm/kill UI", () => {
  const originalWebSocket = globalThis.WebSocket;

  beforeEach(() => {
    MockWebSocket.instances = [];
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket;
    vi.useFakeTimers();
  });

  afterEach(() => {
    cleanup();
    vi.useRealTimers();
    globalThis.WebSocket = originalWebSocket;
    vi.restoreAllMocks();
  });

  it("clicking KILL sends c:kill and shows killed badge", () => {
    render(<ControlPage />);
    connectAndOpen();

    const killBtn = screen.getByRole("button", { name: /KILL \(k\)/ });
    act(() => {
      fireEvent.click(killBtn);
    });

    const calls = lastWs().send.mock.calls.filter(
      (c) => typeof c[0] === "string" && (c[0] as string).startsWith("c:"),
    );
    expect(calls[0][0]).toBe("c:kill");
    const badge = screen.getByTestId("arming-badge");
    expect(badge.textContent).toBe("killed");
  });

  it("clicking Arm while in killed state sends c:clearkill THEN c:arm", () => {
    render(<ControlPage />);
    connectAndOpen();

    // Force into killed state first.
    const killBtn = screen.getByRole("button", { name: /KILL \(k\)/ });
    act(() => {
      fireEvent.click(killBtn);
    });

    // Now click Arm — must send clearkill then arm in order.
    const armBtn = screen.getByRole("button", { name: /Arm \(1\)/ });
    act(() => {
      vi.advanceTimersByTime(100); // clear debounce
      fireEvent.click(armBtn);
    });

    const controlCalls = lastWs().send.mock.calls
      .map((c) => c[0] as string)
      .filter((s) => typeof s === "string" && s.startsWith("c:"));
    // Expect c:kill, c:clearkill, c:arm in that order
    const ck = controlCalls.indexOf("c:clearkill");
    const ar = controlCalls.indexOf("c:arm");
    expect(ck).toBeGreaterThan(-1);
    expect(ar).toBeGreaterThan(ck);
  });

  it("WS drop while armed flips arming to disarmed and shows notice", () => {
    render(<ControlPage />);
    connectAndOpen();

    const armBtn = screen.getByRole("button", { name: /Arm \(1\)/ });
    act(() => {
      fireEvent.click(armBtn);
    });
    // sanity: now armed
    expect(screen.getByTestId("arming-badge").textContent).toBe("armed");

    // Drop the ws
    act(() => {
      MockWebSocket.instances[0].simulateClose();
    });

    expect(screen.getByText(/connection dropped/i)).toBeTruthy();
    expect(screen.getByTestId("arming-badge").textContent).toBe("disarmed");
  });

  it("50ms debounce prevents double-fire on rapid clicks", () => {
    render(<ControlPage />);
    connectAndOpen();

    const armBtn = screen.getByRole("button", { name: /Arm \(1\)/ });
    act(() => {
      fireEvent.click(armBtn);
      fireEvent.click(armBtn);
    });

    const controlCalls = lastWs().send.mock.calls
      .map((c) => c[0] as string)
      .filter((s) => typeof s === "string" && s.startsWith("c:"));
    const armCount = controlCalls.filter((s) => s === "c:arm").length;
    expect(armCount).toBe(1);
  });
});
