/** Developed with AI assistance (Claude, Anthropic) */

import { VideoSource, VideoSourceCallbacks } from "./types";

export class WebSocketSource implements VideoSource {
  readonly type = "websocket";

  private ws: WebSocket | null = null;
  private _connected = false;

  get connected() {
    return this._connected;
  }

  connect(url: string, { onFrame, onError }: VideoSourceCallbacks) {
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
        onFrame(img);
        URL.revokeObjectURL(objectUrl);
      };
      img.onerror = () => {
        URL.revokeObjectURL(objectUrl);
      };
      img.src = objectUrl;
    };

    ws.onerror = () => {
      onError(new Error(`WebSocket stream failed: ${url}`));
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
