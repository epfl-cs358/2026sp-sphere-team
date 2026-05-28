/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import {
  encodeFrame,
  type Cmd,
  type ControlVerb,
  type SocketState,
} from "./types";

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

export type UseTeleopSocketReturn = {
  state: SocketState;
  sendControl: (v: ControlVerb) => "sent" | "dropped";
};

export function useTeleopSocket(opts: {
  url: string | null;
  getCmd: () => Cmd;
  isArmed: () => boolean;
  onDrop?: () => void;
}): UseTeleopSocketReturn {
  const { url, getCmd, isArmed, onDrop } = opts;

  const [phase, setPhase] = useState<Phase | null>(null);

  // Latest-ref consistency rule: inline assignment during render, NEVER
  // useEffect-as-mirror. Reads from these refs inside async callbacks always
  // see the freshest values from the most recent render.
  const getCmdRef = useRef(getCmd);
  getCmdRef.current = getCmd;
  const isArmedRef = useRef(isArmed);
  isArmedRef.current = isArmed;
  const onDropRef = useRef(onDrop);
  onDropRef.current = onDrop;

  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!url) return;

    let ws: WebSocket | null = null;
    let sendInterval: ReturnType<typeof setInterval> | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let stabilityTimer: ReturnType<typeof setTimeout> | null = null;
    let cancelled = false;
    let attempt = 0;
    let wasOpen = false;

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
        wsRef.current = ws;
      } catch {
        scheduleReconnect();
        return;
      }
      ws.onopen = () => {
        if (cancelled) {
          ws?.close();
          return;
        }
        wasOpen = true;
        setPhase({ url, status: "connected" });
        stabilityTimer = setTimeout(() => {
          attempt = 0;
          stabilityTimer = null;
        }, STABILITY_MS);
        sendInterval = setInterval(() => {
          if (!ws || ws.readyState !== WebSocket.OPEN) return;
          if (ws.bufferedAmount > BUFFERED_AMOUNT_LIMIT) return;
          if (!isArmedRef.current()) return;
          const c = getCmdRef.current();
          ws.send(`${c.vx},${c.vy},${c.omega}`);
        }, SEND_RATE_MS);
      };
      ws.onclose = () => {
        clearSend();
        clearStability();
        const dropFired = wasOpen;
        wasOpen = false;
        ws = null;
        if (wsRef.current && wsRef.current.readyState !== WebSocket.OPEN) {
          wsRef.current = null;
        }
        if (cancelled) return;
        if (dropFired) onDropRef.current?.();
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
      wsRef.current = null;
    };
  }, [url]);

  const sendControl = useCallback(
    (verb: ControlVerb): "sent" | "dropped" => {
      const ws = wsRef.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) return "dropped";
      // Intentionally do NOT check bufferedAmount: control verbs (especially
      // kill) must never be dropped due to backpressure.
      ws.send(encodeFrame({ kind: "c", verb }));
      return "sent";
    },
    [],
  );

  let state: SocketState;
  if (!url) state = "idle";
  else if (phase?.url === url) state = phase.status;
  else state = "connecting";

  return { state, sendControl };
}
