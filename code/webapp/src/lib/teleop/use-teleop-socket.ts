/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useEffect, useRef, useState } from "react";
import type { Cmd, SocketState } from "./types";

const SEND_RATE_MS = 50;
// Chip-side WebSocketCommandProducer accepts one client and its heartbeat
// takes ~5s to evict a dead one (enableHeartbeat(2000, 1000, 2)). Retrying
// faster than that just gets the new connection rejected, which closes,
// which schedules another retry — a tight loop that looks like rapid
// disconnect/reconnect flicker. 4s gives the chip time to clear its slot.
const RECONNECT_DELAY_MS = 4000;

type Phase = {
  url: string;
  status: "connected" | "reconnecting";
};

export function useTeleopSocket(opts: {
  url: string | null;
  getCmd: () => Cmd;
}): SocketState {
  const { url, getCmd } = opts;

  // Phase is only ever written from async WebSocket callbacks. The visible
  // state for the "connecting" first-attempt case is derived below from
  // (url is set) AND (phase doesn't match url yet).
  const [phase, setPhase] = useState<Phase | null>(null);

  // Latest-value ref so the 20 Hz send loop always reads the freshest command
  // without re-subscribing the WebSocket every render. Updated post-render.
  const getCmdRef = useRef(getCmd);
  useEffect(() => {
    getCmdRef.current = getCmd;
  });

  useEffect(() => {
    if (!url) return;

    let ws: WebSocket | null = null;
    let sendInterval: ReturnType<typeof setInterval> | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let cancelled = false;

    const clearSend = () => {
      if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
      }
    };

    const scheduleReconnect = () => {
      if (cancelled || reconnectTimer) return;
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        if (!cancelled) open();
      }, RECONNECT_DELAY_MS);
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
        sendInterval = setInterval(() => {
          if (ws && ws.readyState === WebSocket.OPEN) {
            const c = getCmdRef.current();
            ws.send(`${c.vx},${c.vy},${c.omega}`);
          }
        }, SEND_RATE_MS);
      };
      ws.onclose = () => {
        clearSend();
        ws = null;
        if (cancelled) return;
        setPhase({ url, status: "reconnecting" });
        scheduleReconnect();
      };
      // onerror would only race with onclose for the user-visible state;
      // onclose always fires after, so we let it own the transition.
      ws.onerror = () => {};
    };

    open();

    return () => {
      cancelled = true;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
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
