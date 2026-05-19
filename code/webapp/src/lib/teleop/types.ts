/** Developed with AI assistance (Claude, Anthropic) */

export type Cmd = { vx: number; vy: number; omega: number };
export type Maxes = { vx: number; vy: number; omega: number };

export type SocketState = "idle" | "connecting" | "connected" | "reconnecting";

export type ControlVerb = "arm" | "disarm" | "kill" | "clearkill";

export type Frame =
  | { kind: "v"; vx: number; vy: number; omega: number }
  | { kind: "c"; verb: ControlVerb };

export type ArmingState = "disarmed" | "armed" | "killed";

export function encodeFrame(f: Frame): string {
  switch (f.kind) {
    case "v":
      return `${f.vx},${f.vy},${f.omega}`;
    case "c":
      return `c:${f.verb}`;
  }
}
