/** Developed with AI assistance (Claude, Anthropic) */

import { VideoSource, VideoSourceCallbacks } from "./types";

// Backoff matches the teleop hook's design: bias toward fast recovery on
// transient drops, then back off so a dead chip doesn't get hammered.
const BACKOFF_SCHEDULE_MS = [0, 250, 500, 1000, 2000, 4000] as const;
const BACKOFF_CAP_MS = 5000;
const JITTER_FROM_ATTEMPT = 3;
const STABILITY_RESET_MS = 3000;

export class WebSocketSource implements VideoSource {
  readonly type = "websocket";

  private ws: WebSocket | null = null;
  private _connected = false;
  private _attempt = 0;
  private _reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private _stabilityTimer: ReturnType<typeof setTimeout> | null = null;
  private _url: string | null = null;
  private _callbacks: VideoSourceCallbacks | null = null;

  get connected() {
    return this._connected;
  }

  connect(url: string, callbacks: VideoSourceCallbacks) {
    this.disconnect();
    this._url = url;
    this._callbacks = callbacks;
    this._attempt = 0;
    this.openSocket();
  }

  disconnect() {
    this.clearReconnectTimer();
    this.clearStabilityTimer();
    if (this.ws) {
      // Detach handlers first so the trailing close event can't re-enter
      // scheduleReconnect on a torn-down source.
      this.ws.onopen = null;
      this.ws.onmessage = null;
      this.ws.onerror = null;
      this.ws.onclose = null;
      this.ws.close();
      this.ws = null;
    }
    this._url = null;
    this._callbacks = null;
    this._connected = false;
  }

  private openSocket = () => {
    this._reconnectTimer = null;
    const url = this._url;
    const callbacks = this._callbacks;
    if (!url || !callbacks) return;

    const ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
      this._connected = true;
      this.clearReconnectTimer();
      this.clearStabilityTimer();
      this._stabilityTimer = setTimeout(() => {
        this._attempt = 0;
        this._stabilityTimer = null;
      }, STABILITY_RESET_MS);
    };

    ws.onmessage = (event: MessageEvent) => {
      if (!(event.data instanceof ArrayBuffer)) return;

      const blob = new Blob([event.data], { type: "image/jpeg" });
      const objectUrl = URL.createObjectURL(blob);

      const img = new Image();
      img.onload = () => {
        callbacks.onFrame(img);
        URL.revokeObjectURL(objectUrl);
      };
      img.onerror = () => {
        URL.revokeObjectURL(objectUrl);
      };
      img.src = objectUrl;
    };

    ws.onerror = () => {
      callbacks.onError(new Error(`WebSocket stream failed: ${url}`));
    };

    ws.onclose = () => {
      this._connected = false;
      this.clearStabilityTimer();
      // disconnect() nulls _url; that gates any reconnect from in-flight events.
      if (this._url !== url) return;
      this.scheduleReconnect();
    };

    this.ws = ws;
  };

  private scheduleReconnect() {
    if (this._reconnectTimer) return;
    const delay = this.backoffMs(this._attempt++);
    this._reconnectTimer = setTimeout(this.openSocket, delay);
  }

  private backoffMs(attempt: number): number {
    const base =
      attempt < BACKOFF_SCHEDULE_MS.length
        ? BACKOFF_SCHEDULE_MS[attempt]
        : BACKOFF_SCHEDULE_MS[BACKOFF_SCHEDULE_MS.length - 1];
    let delay = base;
    if (attempt >= JITTER_FROM_ATTEMPT) {
      // ±20 %
      const factor = 1 + (Math.random() * 2 - 1) * 0.2;
      delay = base * factor;
    }
    return Math.min(delay, BACKOFF_CAP_MS);
  }

  private clearReconnectTimer() {
    if (this._reconnectTimer) {
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = null;
    }
  }

  private clearStabilityTimer() {
    if (this._stabilityTimer) {
      clearTimeout(this._stabilityTimer);
      this._stabilityTimer = null;
    }
  }
}
