/** Developed with AI assistance (Claude, Anthropic) */

export type Cmd = { vx: number; vy: number; omega: number };
export type Maxes = { vx: number; vy: number; omega: number };

export type SocketState = "idle" | "connecting" | "connected" | "reconnecting";
