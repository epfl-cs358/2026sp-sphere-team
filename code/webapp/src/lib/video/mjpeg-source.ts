/** Developed with AI assistance (Claude, Anthropic) */

import { VideoSource, VideoSourceCallbacks } from "./types";

export class MJPEGSource implements VideoSource {
  readonly type = "mjpeg";

  private img: HTMLImageElement | null = null;
  private _connected = false;

  get connected() {
    return this._connected;
  }

  connect(url: string, { onFrame, onError }: VideoSourceCallbacks) {
    this.disconnect();

    const img = new Image();
    img.crossOrigin = "anonymous";

    img.onload = () => {
      this._connected = true;
      onFrame(img);
    };

    img.onerror = () => {
      this._connected = false;
      onError(new Error(`MJPEG stream failed: ${url}`));
    };

    // MJPEG streams continuously update the same <img> element
    img.src = url;
    this.img = img;
  }

  disconnect() {
    if (this.img) {
      this.img.src = "";
      this.img = null;
    }
    this._connected = false;
  }
}
