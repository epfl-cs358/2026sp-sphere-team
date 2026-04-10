/** Developed with AI assistance (Claude, Anthropic) */

import { VideoSource } from "./types";

export class WebSocketSource implements VideoSource {
  readonly type = "websocket";
  onFrame: ((frame: HTMLImageElement) => void) | null = null;
  onError: ((error: Error) => void) | null = null;

  private ws: WebSocket | null = null;
  private _connected = false;

  get connected() {
    return this._connected;
  }

  connect(url: string) {
    this.disconnect();

    const ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
      this._connected = true;
    };

    ws.onmessage = (event: MessageEvent) => {
      if (!(event.data instanceof ArrayBuffer)) return;

      const blob = new Blob([event.data], { type: "image/jpeg" });
      const objectUrl = URL.createObjectURL(blob);

      const img = new Image();
      img.onload = () => {
        this.onFrame?.(img);
        URL.revokeObjectURL(objectUrl);
      };
      img.onerror = () => {
        URL.revokeObjectURL(objectUrl);
      };
      img.src = objectUrl;
    };

    ws.onerror = () => {
      this.onError?.(new Error(`WebSocket stream failed: ${url}`));
    };

    ws.onclose = () => {
      this._connected = false;
    };

    this.ws = ws;
  }

  disconnect() {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this._connected = false;
  }
}
