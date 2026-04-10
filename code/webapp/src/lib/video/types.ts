/** Developed with AI assistance (Claude, Anthropic) */

export interface VideoSource {
  readonly type: string;
  connect(url: string): void;
  disconnect(): void;
  onFrame: ((frame: HTMLImageElement) => void) | null;
  onError: ((error: Error) => void) | null;
  readonly connected: boolean;
}
