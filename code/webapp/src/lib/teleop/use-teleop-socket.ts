/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useEffect, useRef, useState } from "react";
import type { Cmd, SocketState } from "./types";

const SEND_RATE_MS = 50;
const BUFFERED_AMOUNT_LIMIT = 256;
const STABILITY_MS = 3000;

// Schedule: 0, 250, 500, 1000, 2000, then 4000 capped at 5000.
// Jitter (±20%) kicks in at attempt 3 so short blips still recover instantly
// while real outages don't synchronize-hammer the single-client chip slot.
function backoffMs(attempt: number): number {
  const base =
    attempt <= 0 ? 0
    : attempt === 1 ? 250
    : attempt === 2 ? 500
    : attempt === 3 ? 1000
    : attempt === 4 ? 2000
    : 4000;
  if (attempt < 3) return base;
  const jittered = base * (1 + (Math.random() * 0.4 - 0.2));
  return Math.min(jittered, 5000);
}

type Phase = {
  url: string;
  status: "connected" | "reconnecting";
};

export function useTeleopSocket(opts: {
  url: string | null;
  getCmd: () => Cmd;
}): SocketState {
  const { url, getCmd } = opts;

  const [phase, setPhase] = useState<Phase | null>(null);

  const getCmdRef = useRef(getCmd);
  useEffect(() => {
    getCmdRef.current = getCmd;
  });

  useEffect(() => {
    if (!url) return;

    let ws: WebSocket | null = null;
    let sendInterval: ReturnType<typeof setInterval> | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let stabilityTimer: ReturnType<typeof setTimeout> | null = null;
    let cancelled = false;
    let attempt = 0;

    const clearSend = () => {
      if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
      }
    };

    const clearStability = () => {
      if (stabilityTimer) {
        clearTimeout(stabilityTimer);
        stabilityTimer = null;
      }
    };

    const scheduleReconnect = () => {
      if (cancelled || reconnectTimer) return;
      const delay = backoffMs(attempt);
      attempt += 1;
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        if (!cancelled) open();
      }, delay);
    };

    const open = () => {
      if (cancelled) return;
      try {
        ws = new WebSocket(url);
      } catch {
        scheduleReconnect();
        return;
      }
      ws.onopen = () => {
        if (cancelled) {
          ws?.close();
          return;
        }
        setPhase({ url, status: "connected" });
        stabilityTimer = setTimeout(() => {
          attempt = 0;
          stabilityTimer = null;
        }, STABILITY_MS);
        sendInterval = setInterval(() => {
          if (!ws || ws.readyState !== WebSocket.OPEN) return;
          if (ws.bufferedAmount > BUFFERED_AMOUNT_LIMIT) return;
          const c = getCmdRef.current();
          ws.send(`${c.vx},${c.vy},${c.omega}`);
        }, SEND_RATE_MS);
      };
      ws.onclose = () => {
        clearSend();
        clearStability();
        ws = null;
        if (cancelled) return;
        setPhase({ url, status: "reconnecting" });
        scheduleReconnect();
      };
      ws.onerror = (e) => {
        console.warn("teleop ws error", url, e);
      };
    };

    open();

    return () => {
      cancelled = true;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
      clearStability();
      clearSend();
      if (ws) {
        ws.onopen = null;
        ws.onclose = null;
        ws.onerror = null;
        ws.close();
        ws = null;
      }
    };
  }, [url]);

  if (!url) return "idle";
  if (phase?.url === url) return phase.status;
  return "connecting";
}
