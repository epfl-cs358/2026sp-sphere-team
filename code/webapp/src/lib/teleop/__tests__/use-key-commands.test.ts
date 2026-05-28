/** Developed with AI assistance (Claude, Anthropic) */

import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { renderHook, act } from "@testing-library/react";
import { useKeyCommands } from "../use-key-commands";
import type { ControlVerb, Maxes } from "../types";

const MAXES: Maxes = { vx: 1, vy: 1, omega: 1 };

function dispatchKey(
  type: "keydown" | "keyup",
  key: string,
  init: KeyboardEventInit = {},
) {
  const ev = new KeyboardEvent(type, { key, ...init });
  window.dispatchEvent(ev);
}

describe("useKeyCommands - control hotkeys", () => {
  beforeEach(() => {
    // Focus body so isEditableTarget returns false by default
    if (document.activeElement instanceof HTMLElement) {
      (document.activeElement as HTMLElement).blur();
    }
  });

  afterEach(() => {
    document.body.innerHTML = "";
    vi.restoreAllMocks();
  });

  it("k key calls onControl('kill')", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "k");
    });
    expect(onControl).toHaveBeenCalledTimes(1);
    expect(onControl).toHaveBeenCalledWith("kill");
  });

  it("1 key calls onControl('arm')", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "1");
    });
    expect(onControl).toHaveBeenCalledWith("arm");
  });

  it("0 key calls onControl('disarm')", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "0");
    });
    expect(onControl).toHaveBeenCalledWith("disarm");
  });

  it("k with metaKey does NOT trigger kill", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "k", { metaKey: true });
    });
    expect(onControl).not.toHaveBeenCalled();
  });

  it("k with ctrlKey does NOT trigger kill", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "k", { ctrlKey: true });
    });
    expect(onControl).not.toHaveBeenCalled();
  });

  it("1 in a focused <input> does NOT trigger arm", () => {
    const input = document.createElement("input");
    document.body.appendChild(input);
    input.focus();
    const onControl = vi.fn<(v: ControlVerb) => void>();
    renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "1");
    });
    expect(onControl).not.toHaveBeenCalled();
  });

  it("control hotkeys do not affect velocity cmd output", () => {
    const onControl = vi.fn<(v: ControlVerb) => void>();
    const { result } = renderHook(() => useKeyCommands(MAXES, { onControl }));
    act(() => {
      dispatchKey("keydown", "k");
      dispatchKey("keydown", "1");
      dispatchKey("keydown", "0");
    });
    expect(result.current.cmd).toEqual({ vx: 0, vy: 0, omega: 0 });
  });

  it("works without onControl option (backwards compat: velocity still works)", () => {
    const { result } = renderHook(() => useKeyCommands(MAXES));
    act(() => {
      dispatchKey("keydown", "w");
    });
    expect(result.current.cmd.vx).toBe(1);
  });
});
