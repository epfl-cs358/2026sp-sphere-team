/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useEffect, useRef, useState } from "react";
import type { Cmd, Maxes } from "./types";

const ZERO: Cmd = { vx: 0, vy: 0, omega: 0 };

function isEditableTarget(el: Element | null): boolean {
  if (!el) return false;
  const tag = el.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return true;
  return (el as HTMLElement).isContentEditable === true;
}

export function useKeyCommands(maxes: Maxes): {
  cmd: Cmd;
  cmdRef: React.RefObject<Cmd>;
} {
  const [cmd, setCmd] = useState<Cmd>(ZERO);
  const cmdRef = useRef<Cmd>(ZERO);

  // Latest-value ref so the keydown handler reads fresh slider maxes without
  // re-subscribing every time the user drags. Updated post-render.
  const maxesRef = useRef(maxes);
  useEffect(() => {
    maxesRef.current = maxes;
  });

  useEffect(() => {
    const keys = new Set<string>();

    const recompute = () => {
      const m = maxesRef.current;
      let vx = 0,
        vy = 0,
        omega = 0;
      if (keys.has("ArrowUp") || keys.has("w") || keys.has("W")) vx += m.vx;
      if (keys.has("ArrowDown") || keys.has("s") || keys.has("S")) vx -= m.vx;
      if (keys.has("ArrowLeft") || keys.has("a") || keys.has("A"))
        omega += m.omega;
      if (keys.has("ArrowRight") || keys.has("d") || keys.has("D"))
        omega -= m.omega;
      if (keys.has("q") || keys.has("Q")) vy += m.vy;
      if (keys.has("e") || keys.has("E")) vy -= m.vy;
      if (keys.has(" ")) {
        vx = 0;
        vy = 0;
        omega = 0;
      }
      const next = { vx, vy, omega };
      cmdRef.current = next;
      setCmd(next);
    };

    const onDown = (e: KeyboardEvent) => {
      if (isEditableTarget(document.activeElement)) return;
      if (e.repeat) return;
      keys.add(e.key);
      recompute();
      if (e.key === " " || e.key.startsWith("Arrow")) e.preventDefault();
    };

    const onUp = (e: KeyboardEvent) => {
      // Always clear on keyup, even if focus is in an input — otherwise a key
      // pressed in the page and released over an input would stay "held".
      if (keys.delete(e.key)) recompute();
    };

    const onBlur = () => {
      if (keys.size === 0) return;
      keys.clear();
      recompute();
    };

    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    window.addEventListener("blur", onBlur);
    return () => {
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
      window.removeEventListener("blur", onBlur);
    };
  }, []);

  return { cmd, cmdRef };
}
