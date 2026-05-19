/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useCallback, useRef, useState } from "react";
import {
  useKeyCommands,
  useTeleopSocket,
  type ArmingState,
  type ControlVerb,
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

const ARMING_LABEL: Record<ArmingState, string> = {
  disarmed: "disarmed",
  armed: "armed",
  killed: "killed",
};

const ARMING_CLASS: Record<ArmingState, string> = {
  disarmed: "bg-zinc-900 text-zinc-500",
  armed: "bg-emerald-950 text-emerald-400",
  killed: "bg-red-950 text-red-400 animate-pulse",
};

const DEBOUNCE_MS = 50;

export default function ControlPage() {
  const [host, setHost] = useState("bb8-robot.local:80");
  const [enabled, setEnabled] = useState(false);
  const [vxMax, setVxMax] = useState(0.3);
  const [vyMax, setVyMax] = useState(0.3);
  const [omegaMax, setOmegaMax] = useState(1.0);

  const [arming, setArming] = useState<ArmingState>("disarmed");
  const armingRef = useRef<ArmingState>("disarmed");
  armingRef.current = arming;

  const [notice, setNotice] = useState<string | null>(null);
  const [lastCmd, setLastCmd] = useState<ControlVerb | null>(null);
  const [lastSendAt, setLastSendAt] = useState<number | null>(null);

  const lastClickRef = useRef<number>(0);

  const { cmd, cmdRef } = useKeyCommands(
    { vx: vxMax, vy: vyMax, omega: omegaMax },
    {
      onControl: (v) => {
        if (v === "arm") onArm();
        else if (v === "disarm") onDisarm();
        else if (v === "kill") onKill();
      },
    },
  );

  const getCmd = useCallback(() => cmdRef.current, [cmdRef]);

  const { state, sendControl } = useTeleopSocket({
    url: enabled ? `ws://${host}` : null,
    getCmd,
    isArmed: () => armingRef.current === "armed",
    onDrop: () => {
      if (armingRef.current === "armed") {
        setArming("disarmed");
        setNotice("Connection dropped — re-arm required.");
      }
    },
  });

  const debounced = (fn: () => void) => {
    const now = Date.now();
    if (now - lastClickRef.current < DEBOUNCE_MS) return;
    lastClickRef.current = now;
    fn();
  };

  const tryReportSend = (v: ControlVerb, res: "sent" | "dropped"): boolean => {
    if (res === "dropped") {
      setNotice(`Failed to send c:${v} — WS not open.`);
      return false;
    }
    setLastCmd(v);
    setLastSendAt(Date.now());
    setNotice(null);
    return true;
  };

  function onArm() {
    debounced(() => {
      if (armingRef.current === "killed") {
        const a = sendControl("clearkill");
        if (!tryReportSend("clearkill", a)) return;
        const b = sendControl("arm");
        if (!tryReportSend("arm", b)) return;
      } else {
        const r = sendControl("arm");
        if (!tryReportSend("arm", r)) return;
      }
      setArming("armed");
    });
  }

  function onDisarm() {
    debounced(() => {
      const r = sendControl("disarm");
      if (!tryReportSend("disarm", r)) return;
      setArming("disarmed");
    });
  }

  function onKill() {
    debounced(() => {
      const r = sendControl("kill");
      if (!tryReportSend("kill", r)) return;
      setArming("killed");
    });
  }

  function onClearKill() {
    debounced(() => {
      const r = sendControl("clearkill");
      if (!tryReportSend("clearkill", r)) return;
      setArming("disarmed");
    });
  }

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
          <span
            className={`rounded-md px-3 py-1.5 text-xs font-mono ${ARMING_CLASS[arming]}`}
            data-testid="arming-badge"
          >
            {ARMING_LABEL[arming]}
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

      <div className="flex flex-wrap items-center gap-3">
        <button
          onClick={onArm}
          className="rounded-md bg-emerald-700 hover:bg-emerald-600 active:bg-emerald-800 px-5 py-3 text-sm font-semibold text-white shadow-md"
        >
          Arm (1)
        </button>
        <button
          onClick={onDisarm}
          className="rounded-md bg-zinc-700 hover:bg-zinc-600 active:bg-zinc-800 px-5 py-3 text-sm font-semibold text-white shadow-md"
        >
          Disarm (0)
        </button>
        <button
          onClick={onKill}
          className="rounded-lg bg-red-600 hover:bg-red-500 active:bg-red-700 px-8 py-6 text-2xl font-extrabold tracking-wider text-white shadow-lg ring-4 ring-red-900/60"
        >
          KILL (k)
        </button>
        {arming === "killed" && (
          <button
            onClick={onClearKill}
            className="rounded-md bg-amber-700 hover:bg-amber-600 active:bg-amber-800 px-5 py-3 text-sm font-semibold text-white shadow-md"
          >
            Clear Kill
          </button>
        )}
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
          <KeyHint keys={["k"]} label="KILL" />
          <KeyHint keys={["1"]} label="ARM" />
          <KeyHint keys={["0"]} label="DISARM" />
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

      {/*
        Telemetry panel — local-derived only for now.
        TODO (Ticket A): replace with live firmware status WS for pitch/roll/
        calibration once the dedicated status broadcast lands.
      */}
      <div className="rounded-lg border border-zinc-800 bg-zinc-900 p-6 font-mono text-sm">
        <h2 className="mb-3 text-xs uppercase tracking-wider text-zinc-500">
          Telemetry
        </h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-x-10 gap-y-2">
          <TelemetryRow label="arming" value={ARMING_LABEL[arming]} />
          <TelemetryRow label="ws" value={STATUS_LABEL[state]} />
          <TelemetryRow
            label="last cmd"
            value={lastCmd ? `c:${lastCmd}` : "—"}
          />
          <TelemetryRow
            label="t since last send"
            value={
              lastSendAt
                ? `${Math.max(0, Date.now() - lastSendAt)} ms`
                : "—"
            }
          />
        </div>
      </div>

      {notice && (
        <div className="rounded-md border border-red-800 bg-red-950 px-4 py-2 text-sm text-red-300">
          {notice}
        </div>
      )}

      <p className="text-xs text-zinc-600">
        Velocity sent every 50 ms (20 Hz) while armed. Firmware ramps to zero
        after 200 ms of silence; auto-reconnects on drop.
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

function TelemetryRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between gap-4 border-b border-zinc-800 pb-1 last:border-0">
      <span className="text-zinc-500">{label}</span>
      <span className="text-zinc-200">{value}</span>
    </div>
  );
}
