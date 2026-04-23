/** Developed with AI assistance (Claude, Anthropic) */

export interface VideoSourceCallbacks {
  onFrame: (frame: HTMLImageElement) => void;
  onError: (error: Error) => void;
}

export interface VideoSource {
  readonly type: string;
  connect(url: string, callbacks: VideoSourceCallbacks): void;
  disconnect(): void;
  readonly connected: boolean;
}
