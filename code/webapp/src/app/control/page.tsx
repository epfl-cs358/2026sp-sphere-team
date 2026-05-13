/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useCallback, useState } from "react";
import {
  useKeyCommands,
  useTeleopSocket,
  type SocketState,
} from "@/lib/teleop";

const STATUS_LABEL: Record<SocketState, string> = {
  idle: "disconnected",
  connecting: "connecting",
  connected: "connected",
  reconnecting: "reconnecting",
};

const STATUS_CLASS: Record<SocketState, string> = {
  idle: "bg-zinc-900 text-zinc-500",
  connecting: "bg-yellow-950 text-yellow-400",
  connected: "bg-green-950 text-green-400",
  reconnecting: "bg-yellow-950 text-yellow-400",
};

export default function ControlPage() {
  const [host, setHost] = useState("bb8-robot.local:80");
  const [enabled, setEnabled] = useState(false);
  const [vxMax, setVxMax] = useState(0.3);
  const [vyMax, setVyMax] = useState(0.3);
  const [omegaMax, setOmegaMax] = useState(1.0);

  const { cmd, cmdRef } = useKeyCommands({
    vx: vxMax,
    vy: vyMax,
    omega: omegaMax,
  });

  const getCmd = useCallback(() => cmdRef.current, [cmdRef]);

  const state = useTeleopSocket({
    url: enabled ? `ws://${host}` : null,
    getCmd,
  });

  const toggle = (e: React.MouseEvent<HTMLButtonElement>) => {
    setEnabled((v) => !v);
    // Drop focus so Space (e-stop) or Enter can't re-trigger the button.
    e.currentTarget.blur();
  };

  return (
    <div className="flex flex-1 flex-col gap-6 p-6">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <h1 className="text-xl font-semibold">RC Control</h1>
        <div className="flex flex-wrap items-center gap-3">
          <div className="flex items-center gap-2 rounded-md border border-zinc-800 bg-zinc-900 pl-3">
            <span className="text-xs text-zinc-500 font-mono">ws://</span>
            <input
              type="text"
              value={host}
              onChange={(e) => setHost(e.target.value)}
              disabled={enabled}
              placeholder="bb8-robot.local:80"
              className="bg-transparent px-1 py-1.5 text-sm font-mono text-zinc-100 placeholder:text-zinc-600 w-56 outline-none disabled:opacity-50"
            />
          </div>
          <span
            className={`rounded-md px-3 py-1.5 text-xs font-mono ${STATUS_CLASS[state]}`}
          >
            {STATUS_LABEL[state]}
          </span>
          <button
            onClick={toggle}
            className={`rounded-md px-4 py-1.5 text-sm font-medium transition-colors ${
              enabled
                ? "border border-red-800 bg-red-950 text-red-400 hover:bg-red-900"
                : "bg-zinc-100 text-zinc-900 hover:bg-zinc-200"
            }`}
          >
            {enabled ? "Disconnect" : "Connect"}
          </button>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <SliderRow
          label="vx max"
          unit="m/s"
          min={0.05}
          max={1.0}
          step={0.05}
          value={vxMax}
          onChange={setVxMax}
        />
        <SliderRow
          label="vy max"
          unit="m/s"
          min={0.05}
          max={1.0}
          step={0.05}
          value={vyMax}
          onChange={setVyMax}
        />
        <SliderRow
          label="ω max"
          unit="rad/s"
          min={0.1}
          max={3.0}
          step={0.1}
          value={omegaMax}
          onChange={setOmegaMax}
        />
      </div>

      <div className="rounded-lg border border-zinc-800 bg-zinc-900 p-6">
        <h2 className="mb-3 text-sm font-medium text-zinc-400">Keys</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-x-6 gap-y-2 text-sm">
          <KeyHint keys={["↑", "W"]} label="forward" />
          <KeyHint keys={["↓", "S"]} label="back" />
          <KeyHint keys={["←", "A"]} label="turn left" />
          <KeyHint keys={["→", "D"]} label="turn right" />
          <KeyHint keys={["Q"]} label="strafe left" />
          <KeyHint keys={["E"]} label="strafe right" />
          <div className="sm:col-span-2">
            <KeyHint keys={["Space"]} label="emergency stop (zeros all)" />
          </div>
        </div>
      </div>

      <div className="rounded-lg border border-zinc-800 bg-zinc-900 p-6 font-mono">
        <div className="flex flex-wrap gap-x-10 gap-y-2 text-lg">
          <ReadOut label="vx" value={cmd.vx} />
          <ReadOut label="vy" value={cmd.vy} />
          <ReadOut label="ω" value={cmd.omega} />
        </div>
      </div>

      <p className="text-xs text-zinc-600">
        Sends every 50 ms (20 Hz). Firmware ramps to zero after 200 ms of
        silence; auto-reconnects on drop.
      </p>
    </div>
  );
}

function SliderRow({
  label,
  unit,
  min,
  max,
  step,
  value,
  onChange,
}: {
  label: string;
  unit: string;
  min: number;
  max: number;
  step: number;
  value: number;
  onChange: (v: number) => void;
}) {
  return (
    <label className="flex flex-col gap-2 rounded-lg border border-zinc-800 bg-zinc-900 p-4">
      <span className="text-xs text-zinc-500">
        {label}{" "}
        <span className="text-zinc-200 font-mono">{value.toFixed(2)}</span>{" "}
        {unit}
      </span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        // Release focus after drag so arrow keys go to the robot, not the slider.
        onPointerUp={(e) => e.currentTarget.blur()}
        onKeyUp={(e) => e.currentTarget.blur()}
        className="accent-blue-500"
      />
    </label>
  );
}

function KeyHint({ keys, label }: { keys: string[]; label: string }) {
  return (
    <div className="flex items-center gap-2">
      {keys.map((k, i) => (
        <span key={i} className="flex items-center gap-2">
          {i > 0 && <span className="text-zinc-600">/</span>}
          <kbd className="inline-block min-w-[1.75rem] rounded bg-zinc-800 px-2 py-0.5 text-center font-mono text-xs text-zinc-200">
            {k}
          </kbd>
        </span>
      ))}
      <span className="text-zinc-400">{label}</span>
    </div>
  );
}

function ReadOut({ label, value }: { label: string; value: number }) {
  return (
    <div>
      <span className="mr-3 text-zinc-500">{label}</span>
      <span className="text-blue-400">{value.toFixed(2)}</span>
    </div>
  );
}
