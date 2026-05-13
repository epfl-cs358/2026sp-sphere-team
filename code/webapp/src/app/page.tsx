/** Developed with AI assistance (Claude, Anthropic) */

import Link from "next/link";

export default function Home() {
  return (
    <div className="flex flex-1 items-center justify-center">
      <div className="flex flex-col items-center gap-8">
        <div className="flex flex-col items-center gap-2">
          <h1 className="text-4xl font-bold tracking-tight">BB-8</h1>
          <p className="text-zinc-400">Omnidirectional Spherical Robot</p>
        </div>

        <div className="grid grid-cols-2 gap-4">
          <Link
            href="/video"
            className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-6 hover:border-zinc-600 transition-colors"
          >
            <span className="text-sm font-medium">Video Feed</span>
            <span className="text-xs text-zinc-500">
              Live camera stream from the head
            </span>
          </Link>

          <Link
            href="/control"
            className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-6 hover:border-zinc-600 transition-colors"
          >
            <span className="text-sm font-medium">RC Control</span>
            <span className="text-xs text-zinc-500">
              Keyboard teleop over WebSocket
            </span>
          </Link>

          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-6 opacity-50 cursor-not-allowed">
            <span className="text-sm font-medium">Autonomous</span>
            <span className="text-xs text-zinc-500">Coming soon</span>
          </div>

          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-6 opacity-50 cursor-not-allowed">
            <span className="text-sm font-medium">Telemetry</span>
            <span className="text-xs text-zinc-500">Coming soon</span>
          </div>
        </div>
      </div>
    </div>
  );
}
