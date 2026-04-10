/** Developed with AI assistance (Claude, Anthropic) */

import { VideoSource } from "./types";

export class MockSource implements VideoSource {
  readonly type = "mock";
  onFrame: ((frame: HTMLImageElement) => void) | null = null;
  onError: ((error: Error) => void) | null = null;

  private canvas: HTMLCanvasElement | null = null;
  private animationId: number | null = null;
  private _connected = false;

  get connected() {
    return this._connected;
  }

  connect() {
    this.disconnect();

    const canvas = document.createElement("canvas");
    canvas.width = 320;
    canvas.height = 240;
    this.canvas = canvas;
    this._connected = true;

    const ctx = canvas.getContext("2d");
    let frame = 0;

    const render = () => {
      if (ctx) {
        ctx.fillStyle = "#1a1a2e";
        ctx.fillRect(0, 0, 320, 240);

        ctx.fillStyle = "#e94560";
        const x = 160 + Math.cos(frame * 0.03) * 80;
        const y = 120 + Math.sin(frame * 0.03) * 60;
        ctx.beginPath();
        ctx.arc(x, y, 20, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = "#fff";
        ctx.font = "14px monospace";
        ctx.fillText(`mock frame ${frame}`, 10, 20);
      }
      frame++;

      const img = new Image();
      img.onload = () => this.onFrame?.(img);
      img.src = ctx ? canvas.toDataURL("image/jpeg") : "data:image/jpeg;base64,";

      this.animationId = requestAnimationFrame(render);
    };

    render();
  }

  disconnect() {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    this.canvas = null;
    this._connected = false;
  }
}
