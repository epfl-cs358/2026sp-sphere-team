/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { useTeleopSocket } from "@/lib/teleop";
import type { Cmd } from "@/lib/teleop";

type ConfigKey =
  | "tiltPerVelocity"
  | "maxTiltSetpoint"
  | "pitchKp"
  | "pitchKi"
  | "pitchKd"
  | "rollKp"
  | "rollKi"
  | "rollKd"
  | "pitchDeadband"
  | "rollDeadband"
  | "maxOutputVelocity"
  | "envelopeEnterSin"
  | "envelopeExitSin"
  | "gyroPitchSign"
  | "gyroRollSign";

type BalanceConfig = Record<ConfigKey, number>;

type FieldSpec = {
  key: ConfigKey;
  label: string;
  unit?: string;
  step: number;
  min?: number;
  max?: number;
};

const PITCH_PID: FieldSpec[] = [
  { key: "pitchKp", label: "pitch Kp", step: 0.1, min: 0 },
  { key: "pitchKi", label: "pitch Ki", step: 0.05, min: 0 },
  { key: "pitchKd", label: "pitch Kd", step: 0.01, min: 0 },
];

const ROLL_PID: FieldSpec[] = [
  { key: "rollKp", label: "roll Kp", step: 0.1, min: 0 },
  { key: "rollKi", label: "roll Ki", step: 0.05, min: 0 },
  { key: "rollKd", label: "roll Kd", step: 0.01, min: 0 },
];

const DEADBANDS: FieldSpec[] = [
  { key: "pitchDeadband", label: "pitch deadband", unit: "rad", step: 0.005, min: 0 },
  { key: "rollDeadband",  label: "roll deadband",  unit: "rad", step: 0.005, min: 0 },
];

const COMMAND_MAPPING: FieldSpec[] = [
  {
    key: "tiltPerVelocity",
    label: "tilt per velocity",
    unit: "rad / (m/s)",
    step: 0.05,
    min: 0,
  },
  {
    key: "maxTiltSetpoint",
    label: "max tilt setpoint",
    unit: "rad",
    step: 0.01,
    min: 0,
  },
];

const OUTPUT_SAFETY: FieldSpec[] = [
  {
    key: "maxOutputVelocity",
    label: "max output velocity",
    unit: "m/s",
    step: 0.05,
    min: 0,
  },
];

const FAULT_ENVELOPE: FieldSpec[] = [
  {
    key: "envelopeEnterSin",
    label: "envelope enter sin",
    step: 0.01,
    min: 0,
    max: 1,
  },
  {
    key: "envelopeExitSin",
    label: "envelope exit sin",
    step: 0.01,
    min: 0,
    max: 1,
  },
];

const GYRO_SIGNS: FieldSpec[] = [
  { key: "gyroPitchSign", label: "gyro pitch sign", step: 1, min: -1, max: 1 },
  { key: "gyroRollSign", label: "gyro roll sign", step: 1, min: -1, max: 1 },
];

const GROUPS: { title: string; fields: FieldSpec[] }[] = [
  { title: "Pitch PID", fields: PITCH_PID },
  { title: "Roll PID", fields: ROLL_PID },
  { title: "Deadbands", fields: DEADBANDS },
  { title: "Command Mapping", fields: COMMAND_MAPPING },
  { title: "Output Safety", fields: OUTPUT_SAFETY },
  { title: "Fault Envelope", fields: FAULT_ENVELOPE },
  { title: "Gyro Signs", fields: GYRO_SIGNS },
];

type Status =
  | { kind: "idle" }
  | { kind: "ok" }
  | { kind: "err"; msg: string };

type Toast = { kind: "ok" | "err"; msg: string } | null;

type ArmState = "disarmed" | "armed" | "killed" | "unknown";

export default function TunePage() {
  const [host, setHost] = useState("bb8-robot.local:81");
  const [cfg, setCfg] = useState<BalanceConfig | null>(null);
  const [loadErr, setLoadErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [statusByKey, setStatusByKey] = useState<Record<string, Status>>({});
  const [toast, setToast] = useState<Toast>(null);
  const [armState, setArmState] = useState<ArmState>("unknown");
  const [armErr, setArmErr] = useState<string | null>(null);
  const [vxMax, setVxMax] = useState(0.3);
  const [vyMax, setVyMax] = useState(0.3);
  const [omegaMax, setOmegaMax] = useState(1.0);
  const [padXY, setPadXY] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [omegaNorm, setOmegaNorm] = useState(0);

  const armStateRef = useRef<ArmState>("unknown");
  armStateRef.current = armState;

  const cmdRef = useRef<Cmd>({ vx: 0, vy: 0, omega: 0 });
  cmdRef.current = {
    vx: padXY.y * vxMax,
    vy: padXY.x * vyMax,
    omega: omegaNorm * omegaMax,
  };

  const wsHost = host.replace(/:\d+$/, ":80");
  const { state: wsState } = useTeleopSocket({
    url: `ws://${wsHost}`,
    getCmd: useCallback(() => cmdRef.current, []),
    isArmed: useCallback(() => armStateRef.current === "armed", []),
  });

  // Initial fetch on mount — legitimate effect (one-shot bootstrap).
  useEffect(() => {
    let aborted = false;
    setLoading(true);
    fetch(`http://${host}/balance/config`)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json() as Promise<BalanceConfig>;
      })
      .then((data) => {
        if (aborted) return;
        setCfg(data);
        setLoadErr(null);
      })
      .catch((e: unknown) => {
        if (aborted) return;
        const msg = e instanceof Error ? e.message : String(e);
        setLoadErr(msg);
      })
      .finally(() => {
        if (!aborted) setLoading(false);
      });
    return () => {
      aborted = true;
    };
  }, [host]);

  async function refetchConfig() {
    setLoading(true);
    try {
      const r = await fetch(`http://${host}/balance/config`);
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const data = (await r.json()) as BalanceConfig;
      setCfg(data);
      setLoadErr(null);
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e);
      setLoadErr(msg);
    } finally {
      setLoading(false);
    }
  }

  async function commit(key: ConfigKey, raw: string) {
    const value = parseFloat(raw);
    if (Number.isNaN(value)) {
      setStatusByKey((s) => ({
        ...s,
        [key]: { kind: "err", msg: "not a number" },
      }));
      return;
    }
    try {
      const r = await fetch(`http://${host}/balance/set`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ key, value }),
      });
      const data = (await r.json()) as { ok: boolean; error?: string };
      if (r.ok && data.ok) {
        setStatusByKey((s) => ({ ...s, [key]: { kind: "ok" } }));
        setCfg((c) => (c ? { ...c, [key]: value } : c));
      } else {
        setStatusByKey((s) => ({
          ...s,
          [key]: { kind: "err", msg: data.error ?? `HTTP ${r.status}` },
        }));
      }
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e);
      setStatusByKey((s) => ({ ...s, [key]: { kind: "err", msg } }));
    }
  }

  async function onSave() {
    try {
      const r = await fetch(`http://${host}/balance/save`, { method: "POST" });
      const data = (await r.json()) as { ok: boolean; error?: string };
      if (r.ok && data.ok) {
        setToast({ kind: "ok", msg: "Saved to NVS." });
      } else {
        setToast({ kind: "err", msg: data.error ?? `HTTP ${r.status}` });
      }
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e);
      setToast({ kind: "err", msg });
    }
  }

  // Poll arming state every 1s. Light enough to not flood and short enough to
  // catch fault-gate trips while tuning. Resets whenever the host changes.
  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const poll = async () => {
      try {
        const r = await fetch(`http://${host}/arm/state`);
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const data = (await r.json()) as { state: ArmState };
        if (!cancelled) {
          setArmState(data.state);
          setArmErr(null);
        }
      } catch (e: unknown) {
        if (!cancelled) {
          setArmState("unknown");
          setArmErr(e instanceof Error ? e.message : String(e));
        }
      } finally {
        if (!cancelled) timer = setTimeout(poll, 1000);
      }
    };
    poll();
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, [host]);

  async function sendArmVerb(path: "arm" | "disarm" | "kill" | "clearkill") {
    try {
      const r = await fetch(`http://${host}/${path}`, { method: "POST" });
      const data = (await r.json()) as {
        ok: boolean;
        state?: ArmState;
        error?: string;
      };
      if (r.ok && data.ok) {
        if (data.state) setArmState(data.state);
        setArmErr(null);
      } else {
        setArmErr(data.error ?? `HTTP ${r.status}`);
      }
    } catch (e: unknown) {
      setArmErr(e instanceof Error ? e.message : String(e));
    }
  }

  function snapshotText(c: BalanceConfig): string {
    return (Object.keys(c) as ConfigKey[])
      .map((k) => `balance set ${k} ${c[k]}`)
      .join("\n");
  }

  async function onCopy() {
    if (!cfg) return;
    const text = snapshotText(cfg);
    try {
      await navigator.clipboard.writeText(text);
      setToast({ kind: "ok", msg: "Snapshot copied to clipboard." });
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e);
      setToast({ kind: "err", msg: `Clipboard write failed: ${msg}` });
    }
  }

  async function applyPasted(text: string): Promise<void> {
    // Accepts lines in either form:
    //   balance set <key> <value>
    //   <key>=<value>           (matches `balance show` output)
    //   <key> <value>
    const lines = text
      .split(/\r?\n/)
      .map((l) => l.trim())
      .filter((l) => l.length > 0 && !l.startsWith("#"));
    let ok = 0;
    const errors: string[] = [];
    for (const line of lines) {
      let key: string | null = null;
      let valS: string | null = null;
      const m1 = /^balance\s+set\s+(\S+)\s+(\S+)$/.exec(line);
      const m2 = /^(\S+)\s*=\s*(\S+)$/.exec(line);
      const m3 = /^(\S+)\s+(\S+)$/.exec(line);
      const m = m1 ?? m2 ?? m3;
      if (m) {
        key = m[1];
        valS = m[2];
      }
      if (!key || !valS) {
        errors.push(`unparsable: ${line}`);
        continue;
      }
      const value = parseFloat(valS);
      if (Number.isNaN(value)) {
        errors.push(`${key}: not a number`);
        continue;
      }
      try {
        const r = await fetch(`http://${host}/balance/set`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ key, value }),
        });
        const data = (await r.json()) as { ok: boolean; error?: string };
        if (r.ok && data.ok) {
          ok += 1;
        } else {
          errors.push(`${key}: ${data.error ?? `HTTP ${r.status}`}`);
        }
      } catch (e: unknown) {
        errors.push(`${key}: ${e instanceof Error ? e.message : String(e)}`);
      }
    }
    await refetchConfig();
    if (errors.length === 0) {
      setToast({ kind: "ok", msg: `Applied ${ok} field${ok === 1 ? "" : "s"}.` });
    } else {
      setToast({
        kind: "err",
        msg: `Applied ${ok}; ${errors.length} failed: ${errors.join("; ")}`,
      });
    }
  }

  async function onReset() {
    try {
      const r = await fetch(`http://${host}/balance/reset`, { method: "POST" });
      const data = (await r.json()) as { ok: boolean; error?: string };
      if (r.ok && data.ok) {
        setToast({ kind: "ok", msg: "Reverted to compiled defaults." });
        setStatusByKey({});
        await refetchConfig();
      } else {
        setToast({ kind: "err", msg: data.error ?? `HTTP ${r.status}` });
      }
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e);
      setToast({ kind: "err", msg });
    }
  }

  return (
    <div className="flex flex-1 flex-col gap-6 p-6">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <h1 className="text-xl font-semibold">Balance Tuner</h1>
        <div className="flex flex-wrap items-center gap-3">
          <div className="flex items-center gap-2 rounded-md border border-zinc-800 bg-zinc-900 pl-3">
            <span className="text-xs text-zinc-500 font-mono">http://</span>
            <input
              type="text"
              value={host}
              onChange={(e) => setHost(e.target.value)}
              placeholder="bb8-robot.local:81"
              className="bg-transparent px-1 py-1.5 text-sm font-mono text-zinc-100 placeholder:text-zinc-600 w-56 outline-none"
            />
          </div>
          <button
            onClick={refetchConfig}
            className="rounded-md border border-zinc-700 bg-zinc-800 px-3 py-1.5 text-sm hover:bg-zinc-700"
          >
            Refresh
          </button>
          <button
            onClick={onSave}
            disabled={!cfg}
            className="rounded-md bg-emerald-700 hover:bg-emerald-600 active:bg-emerald-800 px-4 py-1.5 text-sm font-semibold text-white disabled:opacity-40 disabled:cursor-not-allowed"
          >
            Save to NVS
          </button>
          <button
            onClick={onReset}
            disabled={!cfg}
            className="rounded-md bg-amber-700 hover:bg-amber-600 active:bg-amber-800 px-4 py-1.5 text-sm font-semibold text-white disabled:opacity-40 disabled:cursor-not-allowed"
          >
            Reset to defaults
          </button>
        </div>
      </div>

      <ArmingBar
        state={armState}
        err={armErr}
        onArm={() => sendArmVerb("arm")}
        onDisarm={() => sendArmVerb("disarm")}
        onKill={() => sendArmVerb("kill")}
        onClearKill={() => sendArmVerb("clearkill")}
      />

      {loading && (
        <p className="text-sm text-zinc-500">Loading config from {host}…</p>
      )}

      {loadErr && (
        <div className="rounded-md border border-red-800 bg-red-950 px-4 py-2 text-sm text-red-300">
          Failed to load config from {host}: {loadErr}
        </div>
      )}

      {toast && (
        <div
          className={`rounded-md px-4 py-2 text-sm ${
            toast.kind === "ok"
              ? "border border-emerald-800 bg-emerald-950 text-emerald-300"
              : "border border-red-800 bg-red-950 text-red-300"
          }`}
        >
          {toast.msg}{" "}
          <button
            onClick={() => setToast(null)}
            className="ml-2 text-xs underline opacity-70 hover:opacity-100"
          >
            dismiss
          </button>
        </div>
      )}

      {cfg && (
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
          {GROUPS.map((g) => (
            <Group key={g.title} title={g.title}>
              {g.fields.map((f) => (
                <FieldRow
                  key={f.key}
                  spec={f}
                  value={cfg[f.key]}
                  status={statusByKey[f.key] ?? { kind: "idle" }}
                  onCommit={(raw) => commit(f.key, raw)}
                />
              ))}
            </Group>
          ))}
        </div>
      )}

      <TelemetryPanel host={host} />

      <JoystickPanel
        wsState={wsState}
        armed={armState === "armed"}
        wsHost={wsHost}
        vxMax={vxMax}
        vyMax={vyMax}
        omegaMax={omegaMax}
        onVxMax={setVxMax}
        onVyMax={setVyMax}
        onOmegaMax={setOmegaMax}
        padXY={padXY}
        onPadXY={setPadXY}
        omegaNorm={omegaNorm}
        onOmegaNorm={setOmegaNorm}
        cmd={cmdRef.current}
      />

      {cfg && (
        <SnapshotPanel
          text={snapshotText(cfg)}
          onCopy={onCopy}
          onApply={applyPasted}
        />
      )}

      <p className="text-xs text-zinc-600">
        Each field commits on blur or Enter via POST /balance/set. The robot
        hot-swaps the live config atomically; Save to NVS persists across
        reboots.
      </p>
    </div>
  );
}

function JoystickPanel({
  wsState,
  armed,
  wsHost,
  vxMax,
  vyMax,
  omegaMax,
  onVxMax,
  onVyMax,
  onOmegaMax,
  padXY,
  onPadXY,
  omegaNorm,
  onOmegaNorm,
  cmd,
}: {
  wsState: "idle" | "connecting" | "connected" | "reconnecting";
  armed: boolean;
  wsHost: string;
  vxMax: number;
  vyMax: number;
  omegaMax: number;
  onVxMax: (v: number) => void;
  onVyMax: (v: number) => void;
  onOmegaMax: (v: number) => void;
  padXY: { x: number; y: number };
  onPadXY: (v: { x: number; y: number }) => void;
  omegaNorm: number;
  onOmegaNorm: (v: number) => void;
  cmd: Cmd;
}) {
  const wsLabel: Record<typeof wsState, string> = {
    idle: "idle",
    connecting: "connecting",
    connected: "connected",
    reconnecting: "reconnecting",
  };
  const wsClass: Record<typeof wsState, string> = {
    idle: "bg-zinc-800 text-zinc-400",
    connecting: "bg-yellow-950 text-yellow-300",
    connected: "bg-emerald-950 text-emerald-300",
    reconnecting: "bg-yellow-950 text-yellow-300",
  };
  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900 p-4">
      <div className="mb-3 flex flex-wrap items-center gap-3">
        <h2 className="text-xs uppercase tracking-wider text-zinc-500">
          Joystick
        </h2>
        <span
          className={`rounded-md px-2 py-0.5 text-xs font-mono ${wsClass[wsState]}`}
          title={`ws://${wsHost}`}
        >
          ws {wsLabel[wsState]}
        </span>
        {!armed && (
          <span className="rounded-md bg-zinc-800 px-2 py-0.5 text-xs font-mono text-zinc-500">
            (arm to send)
          </span>
        )}
      </div>
      <div className="flex flex-wrap gap-6">
        <div className="flex flex-col items-center gap-2">
          <JoystickPad value={padXY} onChange={onPadXY} />
          <div className="text-xs font-mono text-zinc-500">drag — vx / vy</div>
        </div>
        <div className="flex-1 min-w-[240px] flex flex-col gap-4">
          <OmegaSlider value={omegaNorm} onChange={onOmegaNorm} />
          <div className="grid grid-cols-3 gap-3">
            <MaxField label="vx max" unit="m/s" value={vxMax} step={0.05} onChange={onVxMax} />
            <MaxField label="vy max" unit="m/s" value={vyMax} step={0.05} onChange={onVyMax} />
            <MaxField label="omega max" unit="rad/s" value={omegaMax} step={0.1} onChange={onOmegaMax} />
          </div>
          <div className="rounded-md border border-zinc-800 bg-zinc-950 p-3 font-mono text-xs text-zinc-300">
            <div>vx    = {cmd.vx.toFixed(3)} m/s</div>
            <div>vy    = {cmd.vy.toFixed(3)} m/s</div>
            <div>omega = {cmd.omega.toFixed(3)} rad/s</div>
          </div>
        </div>
      </div>
    </section>
  );
}

const PAD_RADIUS = 100;

function JoystickPad({
  value,
  onChange,
}: {
  value: { x: number; y: number };
  onChange: (v: { x: number; y: number }) => void;
}) {
  const ref = useRef<HTMLDivElement | null>(null);
  const activePointer = useRef<number | null>(null);

  const updateFromEvent = (clientX: number, clientY: number) => {
    const el = ref.current;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = (clientX - cx) / PAD_RADIUS;
    let dy = -(clientY - cy) / PAD_RADIUS;
    const mag = Math.hypot(dx, dy);
    if (mag > 1) {
      dx /= mag;
      dy /= mag;
    }
    onChange({ x: dx, y: dy });
  };

  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (activePointer.current !== null) return;
    activePointer.current = e.pointerId;
    e.currentTarget.setPointerCapture(e.pointerId);
    updateFromEvent(e.clientX, e.clientY);
  };
  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (activePointer.current !== e.pointerId) return;
    updateFromEvent(e.clientX, e.clientY);
  };
  const onPointerEnd = (e: React.PointerEvent<HTMLDivElement>) => {
    if (activePointer.current !== e.pointerId) return;
    activePointer.current = null;
    onChange({ x: 0, y: 0 });
  };

  const knobX = value.x * PAD_RADIUS;
  const knobY = -value.y * PAD_RADIUS;

  return (
    <div
      ref={ref}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerEnd}
      onPointerCancel={onPointerEnd}
      className="relative h-[220px] w-[220px] rounded-full border border-zinc-700 bg-zinc-950 touch-none select-none"
      style={{ cursor: "grab" }}
    >
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute left-1/2 top-0 h-full w-px -translate-x-1/2 bg-zinc-800" />
        <div className="absolute top-1/2 left-0 h-px w-full -translate-y-1/2 bg-zinc-800" />
      </div>
      <div
        className="pointer-events-none absolute h-10 w-10 rounded-full border border-blue-400 bg-blue-600/40"
        style={{
          left: `calc(50% + ${knobX}px - 20px)`,
          top: `calc(50% + ${knobY}px - 20px)`,
        }}
      />
    </div>
  );
}

function OmegaSlider({
  value,
  onChange,
}: {
  value: number;
  onChange: (v: number) => void;
}) {
  return (
    <label className="flex flex-col gap-1">
      <span className="flex items-baseline justify-between text-xs font-mono text-zinc-400">
        <span>omega</span>
        <span className="text-zinc-500">{value.toFixed(2)}</span>
      </span>
      <input
        type="range"
        min={-1}
        max={1}
        step={0.01}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        onPointerUp={() => onChange(0)}
        onPointerCancel={() => onChange(0)}
        onBlur={() => onChange(0)}
        className="w-full accent-blue-500"
      />
    </label>
  );
}

function MaxField({
  label,
  unit,
  value,
  step,
  onChange,
}: {
  label: string;
  unit: string;
  value: number;
  step: number;
  onChange: (v: number) => void;
}) {
  return (
    <label className="flex flex-col gap-1">
      <span className="flex items-baseline justify-between text-xs font-mono text-zinc-400">
        <span>{label}</span>
        <span className="text-zinc-600">{unit}</span>
      </span>
      <input
        type="number"
        step={step}
        min={0}
        value={value}
        onChange={(e) => {
          const v = parseFloat(e.target.value);
          if (!Number.isNaN(v)) onChange(v);
        }}
        className="rounded-md border border-zinc-700 bg-zinc-950 px-2 py-1 text-sm font-mono text-zinc-100 outline-none focus:border-blue-500"
      />
    </label>
  );
}

type TelemetrySnap = {
  seq: number;
  t_us: number;
  dt_measured: number;
  dt_used: number;
  cmd_vx_raw: number;
  cmd_vy_raw: number;
  cmd_omega_raw: number;
  cmd_vx: number;
  cmd_vy: number;
  cmd_omega: number;
  cmd_age_ms: number;
  quat_w: number;
  quat_x: number;
  quat_y: number;
  quat_z: number;
  accel_x: number;
  accel_y: number;
  accel_z: number;
  gyro_x_raw: number;
  gyro_y_raw: number;
  gyro_z_raw: number;
  gx: number;
  gy: number;
  gz: number;
  tilt_mag_sin: number;
  pitch_actual: number;
  roll_actual: number;
  gyro_pitch_rate: number;
  gyro_roll_rate: number;
  pitch_target: number;
  roll_target: number;
  pitch_err: number;
  pitch_P: number;
  pitch_I: number;
  pitch_D: number;
  pitch_out_raw: number;
  pitch_out: number;
  roll_err: number;
  roll_P: number;
  roll_I: number;
  roll_D: number;
  roll_out_raw: number;
  roll_out: number;
  body_vx_cmd: number;
  body_vy_cmd: number;
  body_omega_cmd: number;
  wheel_target_rpm_0: number;
  wheel_target_rpm_1: number;
  wheel_target_rpm_2: number;
  wheel_meas_rpm_0: number;
  wheel_meas_rpm_1: number;
  wheel_meas_rpm_2: number;
  wheel_P_0: number;
  wheel_P_1: number;
  wheel_P_2: number;
  wheel_I_0: number;
  wheel_I_1: number;
  wheel_I_2: number;
  wheel_D_0: number;
  wheel_D_1: number;
  wheel_D_2: number;
  wheel_out_0: number;
  wheel_out_1: number;
  wheel_out_2: number;
  pitch_Kp: number;
  pitch_Ki: number;
  pitch_Kd: number;
  roll_Kp: number;
  roll_Ki: number;
  roll_Kd: number;
  pitch_deadband: number;
  roll_deadband: number;
  max_output_velocity: number;
  envelope_enter_sin: number;
  envelope_exit_sin: number;
  gyro_pitch_sign: number;
  gyro_roll_sign: number;
  tilt_per_velocity: number;
  max_tilt_setpoint: number;
  armed_state: number;
  in_fault: number;
  cmd_stale: number;
  event_flags: number;
};

const EVENT_BITS: { bit: number; name: string }[] = [
  { bit: 0, name: "ARMED_EDGE" },
  { bit: 1, name: "DISARMED_EDGE" },
  { bit: 2, name: "KILLED_EDGE" },
  { bit: 3, name: "KILL_CLEARED" },
  { bit: 4, name: "FAULT_ENTER" },
  { bit: 5, name: "FAULT_EXIT" },
  { bit: 6, name: "PITCH_DEADBAND_RESET" },
  { bit: 7, name: "ROLL_DEADBAND_RESET" },
  { bit: 8, name: "PITCH_I_SATURATED" },
  { bit: 9, name: "ROLL_I_SATURATED" },
  { bit: 10, name: "PITCH_OUT_SATURATED" },
  { bit: 11, name: "ROLL_OUT_SATURATED" },
  { bit: 12, name: "GAIN_CHANGED" },
  { bit: 13, name: "CONFIG_SAVED" },
  { bit: 14, name: "CONFIG_RESET" },
  { bit: 15, name: "STEP_INJECTED" },
];

const ARMED_LABEL: Record<number, string> = {
  0: "disarmed",
  1: "armed",
  2: "killed",
};

function fmt(n: number, digits = 3): string {
  if (!Number.isFinite(n)) return "—";
  return n.toFixed(digits);
}

function TelemetryPanel({ host }: { host: string }) {
  const [snap, setSnap] = useState<TelemetrySnap | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [paused, setPaused] = useState(false);
  const [chipHz, setChipHz] = useState(0);
  const [accumEvents, setAccumEvents] = useState(0);
  const lastSeqRef = useRef<number | null>(null);
  const lastTimeRef = useRef<number | null>(null);

  useEffect(() => {
    if (paused) return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const tick = async () => {
      try {
        const r = await fetch(`http://${host}/telemetry/latest`);
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const data = (await r.json()) as TelemetrySnap;
        if (cancelled) return;
        const now = performance.now();
        if (lastSeqRef.current !== null && lastTimeRef.current !== null) {
          const dseq = data.seq - lastSeqRef.current;
          const dt = (now - lastTimeRef.current) / 1000;
          if (dt > 0 && dseq >= 0) setChipHz(dseq / dt);
        }
        lastSeqRef.current = data.seq;
        lastTimeRef.current = now;
        setSnap(data);
        setAccumEvents((a) => a | data.event_flags);
        setErr(null);
      } catch (e: unknown) {
        if (!cancelled) setErr(e instanceof Error ? e.message : String(e));
      } finally {
        if (!cancelled) timer = setTimeout(tick, 100);
      }
    };
    tick();
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, [host, paused]);

  const armedClass =
    snap?.armed_state === 1
      ? "bg-emerald-950 text-emerald-300 border-emerald-800"
      : snap?.armed_state === 2
        ? "bg-red-950 text-red-300 border-red-800 animate-pulse"
        : "bg-zinc-800 text-zinc-400 border-zinc-700";

  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900 p-4">
      <div className="mb-3 flex flex-wrap items-center gap-3">
        <h2 className="text-xs uppercase tracking-wider text-zinc-500">
          Telemetry
        </h2>
        <span
          className={`rounded-md border px-2 py-0.5 text-xs font-mono ${armedClass}`}
        >
          {snap ? (ARMED_LABEL[snap.armed_state] ?? "?") : "—"}
        </span>
        {snap?.in_fault ? (
          <span className="rounded-md border border-red-800 bg-red-950 px-2 py-0.5 text-xs font-mono text-red-300">
            in_fault
          </span>
        ) : (
          <span className="rounded-md border border-zinc-800 bg-zinc-900 px-2 py-0.5 text-xs font-mono text-zinc-600">
            fault_ok
          </span>
        )}
        {snap?.cmd_stale ? (
          <span className="rounded-md border border-amber-800 bg-amber-950 px-2 py-0.5 text-xs font-mono text-amber-300">
            cmd_stale
          </span>
        ) : (
          <span className="rounded-md border border-zinc-800 bg-zinc-900 px-2 py-0.5 text-xs font-mono text-zinc-600">
            cmd_fresh
          </span>
        )}
        <span className="rounded-md bg-zinc-800 px-2 py-0.5 text-xs font-mono text-zinc-400">
          seq #{snap?.seq ?? 0}
        </span>
        <span className="rounded-md bg-zinc-800 px-2 py-0.5 text-xs font-mono text-zinc-400">
          dt {snap ? (snap.dt_measured * 1000).toFixed(2) : "—"} ms
        </span>
        <span className="rounded-md bg-zinc-800 px-2 py-0.5 text-xs font-mono text-zinc-400">
          chip {chipHz.toFixed(0)} Hz
        </span>
        <span className="rounded-md bg-zinc-800 px-2 py-0.5 text-xs font-mono text-zinc-400">
          cmd_age {snap?.cmd_age_ms ?? 0} ms
        </span>
        <button
          onClick={() => setPaused((p) => !p)}
          className="ml-auto rounded-md border border-zinc-700 bg-zinc-800 hover:bg-zinc-700 px-3 py-1 text-xs font-semibold text-zinc-100"
        >
          {paused ? "Resume" : "Pause"}
        </button>
      </div>

      {err && (
        <div className="mb-3 rounded-md border border-red-800 bg-red-950 px-3 py-1.5 text-xs font-mono text-red-300">
          {err}
        </div>
      )}

      {!snap ? (
        <p className="text-xs text-zinc-500">Waiting for /telemetry/latest…</p>
      ) : (
        <div className="grid grid-cols-1 gap-4 lg:grid-cols-2 xl:grid-cols-3">
          <SubGroup title="Pitch axis">
            <KV label="target" value={fmt(snap.pitch_target, 4)} unit="rad" />
            <KV label="actual" value={fmt(snap.pitch_actual, 4)} unit="rad" />
            <KV label="err" value={fmt(snap.pitch_err, 4)} unit="rad" />
            <KV label="gyro_rate" value={fmt(snap.gyro_pitch_rate, 4)} unit="rad/s" />
            <KV label="P" value={fmt(snap.pitch_P, 4)} />
            <KV label="I" value={fmt(snap.pitch_I, 4)} />
            <KV label="D" value={fmt(snap.pitch_D, 4)} />
            <KV label="out_raw" value={fmt(snap.pitch_out_raw, 4)} />
            <KV label="out" value={fmt(snap.pitch_out, 4)} unit="m/s" />
          </SubGroup>

          <SubGroup title="Roll axis">
            <KV label="target" value={fmt(snap.roll_target, 4)} unit="rad" />
            <KV label="actual" value={fmt(snap.roll_actual, 4)} unit="rad" />
            <KV label="err" value={fmt(snap.roll_err, 4)} unit="rad" />
            <KV label="gyro_rate" value={fmt(snap.gyro_roll_rate, 4)} unit="rad/s" />
            <KV label="P" value={fmt(snap.roll_P, 4)} />
            <KV label="I" value={fmt(snap.roll_I, 4)} />
            <KV label="D" value={fmt(snap.roll_D, 4)} />
            <KV label="out_raw" value={fmt(snap.roll_out_raw, 4)} />
            <KV label="out" value={fmt(snap.roll_out, 4)} unit="m/s" />
          </SubGroup>

          <SubGroup title="Body cmd → drivetrain">
            <KV label="body_vx_cmd" value={fmt(snap.body_vx_cmd, 4)} unit="m/s" />
            <KV label="body_vy_cmd" value={fmt(snap.body_vy_cmd, 4)} unit="m/s" />
            <KV label="body_omega_cmd" value={fmt(snap.body_omega_cmd, 4)} unit="rad/s" />
          </SubGroup>

          <SubGroup title="Operator cmd (raw)">
            <KV label="cmd_vx_raw" value={fmt(snap.cmd_vx_raw, 4)} unit="m/s" />
            <KV label="cmd_vy_raw" value={fmt(snap.cmd_vy_raw, 4)} unit="m/s" />
            <KV label="cmd_omega_raw" value={fmt(snap.cmd_omega_raw, 4)} unit="rad/s" />
          </SubGroup>

          <SubGroup title="Operator cmd (post-ramp)">
            <KV label="cmd_vx" value={fmt(snap.cmd_vx, 4)} unit="m/s" />
            <KV label="cmd_vy" value={fmt(snap.cmd_vy, 4)} unit="m/s" />
            <KV label="cmd_omega" value={fmt(snap.cmd_omega, 4)} unit="rad/s" />
            <KV label="cmd_age_ms" value={String(snap.cmd_age_ms)} unit="ms" />
          </SubGroup>

          <SubGroup title="Orientation">
            <KV label="gx" value={fmt(snap.gx, 4)} />
            <KV label="gy" value={fmt(snap.gy, 4)} />
            <KV label="gz" value={fmt(snap.gz, 4)} />
            <KV label="tilt_mag_sin" value={fmt(snap.tilt_mag_sin, 4)} />
          </SubGroup>

          <SubGroup title="IMU quaternion">
            <KV label="quat_w" value={fmt(snap.quat_w, 4)} />
            <KV label="quat_x" value={fmt(snap.quat_x, 4)} />
            <KV label="quat_y" value={fmt(snap.quat_y, 4)} />
            <KV label="quat_z" value={fmt(snap.quat_z, 4)} />
          </SubGroup>

          <SubGroup title="IMU accel (gravity-removed)">
            <KV label="accel_x" value={fmt(snap.accel_x, 3)} unit="m/s²" />
            <KV label="accel_y" value={fmt(snap.accel_y, 3)} unit="m/s²" />
            <KV label="accel_z" value={fmt(snap.accel_z, 3)} unit="m/s²" />
          </SubGroup>

          <SubGroup title="IMU gyro (raw)">
            <KV label="gyro_x_raw" value={fmt(snap.gyro_x_raw, 4)} unit="rad/s" />
            <KV label="gyro_y_raw" value={fmt(snap.gyro_y_raw, 4)} unit="rad/s" />
            <KV label="gyro_z_raw" value={fmt(snap.gyro_z_raw, 4)} unit="rad/s" />
          </SubGroup>

          <div className="lg:col-span-2 xl:col-span-3">
            <SubGroup title="Wheels">
              <WheelTable snap={snap} />
            </SubGroup>
          </div>

          <SubGroup title="Live gains — pitch">
            <KV label="pitch_Kp" value={fmt(snap.pitch_Kp, 4)} />
            <KV label="pitch_Ki" value={fmt(snap.pitch_Ki, 4)} />
            <KV label="pitch_Kd" value={fmt(snap.pitch_Kd, 4)} />
            <KV label="pitch_deadband" value={fmt(snap.pitch_deadband, 4)} unit="rad" />
          </SubGroup>

          <SubGroup title="Live gains — roll">
            <KV label="roll_Kp" value={fmt(snap.roll_Kp, 4)} />
            <KV label="roll_Ki" value={fmt(snap.roll_Ki, 4)} />
            <KV label="roll_Kd" value={fmt(snap.roll_Kd, 4)} />
            <KV label="roll_deadband" value={fmt(snap.roll_deadband, 4)} unit="rad" />
          </SubGroup>

          <SubGroup title="Live config">
            <KV label="max_output_velocity" value={fmt(snap.max_output_velocity, 3)} unit="m/s" />
            <KV label="envelope_enter_sin" value={fmt(snap.envelope_enter_sin, 3)} />
            <KV label="envelope_exit_sin" value={fmt(snap.envelope_exit_sin, 3)} />
            <KV label="gyro_pitch_sign" value={fmt(snap.gyro_pitch_sign, 0)} />
            <KV label="gyro_roll_sign" value={fmt(snap.gyro_roll_sign, 0)} />
            <KV label="tilt_per_velocity" value={fmt(snap.tilt_per_velocity, 3)} unit="rad/(m/s)" />
            <KV label="max_tilt_setpoint" value={fmt(snap.max_tilt_setpoint, 3)} unit="rad" />
          </SubGroup>

          <SubGroup title="Timing">
            <KV label="seq" value={String(snap.seq)} />
            <KV label="t_us" value={String(snap.t_us)} unit="µs" />
            <KV label="dt_measured" value={fmt(snap.dt_measured * 1000, 3)} unit="ms" />
            <KV label="dt_used" value={fmt(snap.dt_used * 1000, 3)} unit="ms" />
          </SubGroup>

          <div className="lg:col-span-2 xl:col-span-3">
            <SubGroup title="Events (this tick / accumulated)">
              <EventBitsRow
                label="this tick"
                flags={snap.event_flags}
              />
              <EventBitsRow
                label="accumulated"
                flags={accumEvents}
                onReset={() => setAccumEvents(0)}
              />
            </SubGroup>
          </div>
        </div>
      )}
    </section>
  );
}

function SubGroup({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div className="rounded-md border border-zinc-800 bg-zinc-950 p-3">
      <h3 className="mb-2 text-[10px] uppercase tracking-wider text-zinc-500">
        {title}
      </h3>
      <div className="flex flex-col gap-1">{children}</div>
    </div>
  );
}

function KV({
  label,
  value,
  unit,
}: {
  label: string;
  value: string;
  unit?: string;
}) {
  return (
    <div className="flex items-baseline justify-between gap-2 font-mono text-xs">
      <span className="text-zinc-500 truncate">{label}</span>
      <span className="flex items-baseline gap-1">
        <span className="text-zinc-100 tabular-nums">{value}</span>
        {unit && <span className="text-zinc-600">{unit}</span>}
      </span>
    </div>
  );
}

function WheelTable({ snap }: { snap: TelemetrySnap }) {
  const rows: { label: string; values: number[]; unit?: string; digits?: number }[] = [
    {
      label: "target_rpm",
      values: [snap.wheel_target_rpm_0, snap.wheel_target_rpm_1, snap.wheel_target_rpm_2],
      unit: "rpm",
      digits: 2,
    },
    {
      label: "meas_rpm",
      values: [snap.wheel_meas_rpm_0, snap.wheel_meas_rpm_1, snap.wheel_meas_rpm_2],
      unit: "rpm",
      digits: 2,
    },
    {
      label: "P",
      values: [snap.wheel_P_0, snap.wheel_P_1, snap.wheel_P_2],
      digits: 4,
    },
    {
      label: "I",
      values: [snap.wheel_I_0, snap.wheel_I_1, snap.wheel_I_2],
      digits: 4,
    },
    {
      label: "D",
      values: [snap.wheel_D_0, snap.wheel_D_1, snap.wheel_D_2],
      digits: 4,
    },
    {
      label: "out",
      values: [snap.wheel_out_0, snap.wheel_out_1, snap.wheel_out_2],
      digits: 4,
    },
  ];
  return (
    <table className="w-full font-mono text-xs">
      <thead>
        <tr className="text-zinc-500">
          <th className="text-left font-normal pb-1"> </th>
          <th className="text-right font-normal pb-1">w0</th>
          <th className="text-right font-normal pb-1">w1</th>
          <th className="text-right font-normal pb-1">w2</th>
          <th className="text-left font-normal pb-1 pl-2 text-zinc-600"> </th>
        </tr>
      </thead>
      <tbody>
        {rows.map((r) => (
          <tr key={r.label}>
            <td className="py-0.5 text-zinc-500">{r.label}</td>
            {r.values.map((v, i) => (
              <td key={i} className="py-0.5 text-right text-zinc-100 tabular-nums">
                {fmt(v, r.digits ?? 3)}
              </td>
            ))}
            <td className="py-0.5 pl-2 text-zinc-600">{r.unit ?? ""}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

function EventBitsRow({
  label,
  flags,
  onReset,
}: {
  label: string;
  flags: number;
  onReset?: () => void;
}) {
  const active = EVENT_BITS.filter((e) => (flags & (1 << e.bit)) !== 0);
  return (
    <div className="flex items-start gap-2">
      <span className="font-mono text-xs text-zinc-500 shrink-0 w-24">{label}</span>
      <div className="flex flex-wrap gap-1">
        {active.length === 0 ? (
          <span className="font-mono text-xs text-zinc-700">—</span>
        ) : (
          active.map((e) => (
            <span
              key={e.bit}
              className="rounded-sm border border-zinc-700 bg-zinc-800 px-1.5 py-0.5 text-[10px] font-mono text-zinc-200"
            >
              {e.name}
            </span>
          ))
        )}
      </div>
      {onReset && (
        <button
          onClick={onReset}
          className="ml-auto text-[10px] text-zinc-500 hover:text-zinc-300 underline"
        >
          reset
        </button>
      )}
    </div>
  );
}

function SnapshotPanel({
  text,
  onCopy,
  onApply,
}: {
  text: string;
  onCopy: () => void;
  onApply: (text: string) => void | Promise<void>;
}) {
  const [draft, setDraft] = useState("");
  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900 p-4">
      <div className="mb-3 flex items-center justify-between gap-2">
        <h2 className="text-xs uppercase tracking-wider text-zinc-500">
          Snapshot
        </h2>
        <button
          onClick={onCopy}
          className="rounded-md border border-zinc-700 bg-zinc-800 hover:bg-zinc-700 px-3 py-1 text-xs font-semibold text-zinc-100"
        >
          Copy current
        </button>
      </div>
      <pre className="mb-3 max-h-48 overflow-auto rounded-md border border-zinc-800 bg-zinc-950 p-3 text-xs font-mono text-zinc-300 whitespace-pre">
        {text}
      </pre>
      <div className="flex flex-col gap-2">
        <label className="text-xs text-zinc-500">
          Paste a snapshot below and click Apply to restore.
        </label>
        <textarea
          value={draft}
          onChange={(e) => setDraft(e.target.value)}
          rows={6}
          placeholder={"balance set pitchKp 0.8\nbalance set pitchKd 0.07\n…"}
          className="w-full rounded-md border border-zinc-700 bg-zinc-950 px-3 py-2 text-xs font-mono text-zinc-100 outline-none focus:border-blue-500"
        />
        <div>
          <button
            onClick={() => {
              if (draft.trim().length === 0) return;
              onApply(draft);
            }}
            disabled={draft.trim().length === 0}
            className="rounded-md bg-blue-700 hover:bg-blue-600 active:bg-blue-800 px-4 py-1.5 text-sm font-semibold text-white disabled:opacity-40 disabled:cursor-not-allowed"
          >
            Apply pasted
          </button>
        </div>
      </div>
    </section>
  );
}

const ARM_STATE_CLASS: Record<ArmState, string> = {
  disarmed: "bg-zinc-800 text-zinc-300 border-zinc-700",
  armed: "bg-emerald-950 text-emerald-300 border-emerald-800",
  killed: "bg-red-950 text-red-300 border-red-800 animate-pulse",
  unknown: "bg-zinc-900 text-zinc-500 border-zinc-800",
};

function ArmingBar({
  state,
  err,
  onArm,
  onDisarm,
  onKill,
  onClearKill,
}: {
  state: ArmState;
  err: string | null;
  onArm: () => void;
  onDisarm: () => void;
  onKill: () => void;
  onClearKill: () => void;
}) {
  return (
    <div className="flex flex-wrap items-center gap-3 rounded-lg border border-zinc-800 bg-zinc-900 px-4 py-3">
      <span className="text-xs uppercase tracking-wider text-zinc-500">
        Arming
      </span>
      <span
        className={`rounded-md border px-2 py-0.5 text-xs font-mono ${ARM_STATE_CLASS[state]}`}
      >
        {state}
      </span>
      <div className="ml-auto flex flex-wrap items-center gap-2">
        <button
          onClick={onArm}
          disabled={state === "armed" || state === "killed"}
          className="rounded-md bg-emerald-700 hover:bg-emerald-600 active:bg-emerald-800 px-4 py-1.5 text-sm font-semibold text-white disabled:opacity-40 disabled:cursor-not-allowed"
        >
          Arm
        </button>
        <button
          onClick={onDisarm}
          disabled={state === "disarmed" || state === "killed"}
          className="rounded-md border border-zinc-700 bg-zinc-800 hover:bg-zinc-700 px-4 py-1.5 text-sm font-semibold text-zinc-100 disabled:opacity-40 disabled:cursor-not-allowed"
        >
          Disarm
        </button>
        {state === "killed" ? (
          <button
            onClick={onClearKill}
            className="rounded-md bg-amber-700 hover:bg-amber-600 active:bg-amber-800 px-4 py-1.5 text-sm font-semibold text-white"
          >
            Clear Kill
          </button>
        ) : (
          <button
            onClick={onKill}
            className="rounded-md bg-red-700 hover:bg-red-600 active:bg-red-800 px-4 py-1.5 text-sm font-semibold text-white"
          >
            Kill
          </button>
        )}
      </div>
      {err && (
        <span className="w-full text-xs text-red-400 font-mono">{err}</span>
      )}
    </div>
  );
}

function Group({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <section className="rounded-lg border border-zinc-800 bg-zinc-900 p-4">
      <h2 className="mb-3 text-xs uppercase tracking-wider text-zinc-500">
        {title}
      </h2>
      <div className="flex flex-col gap-2">{children}</div>
    </section>
  );
}

function FieldRow({
  spec,
  value,
  status,
  onCommit,
}: {
  spec: FieldSpec;
  value: number;
  status: Status;
  onCommit: (raw: string) => void;
}) {
  const [draft, setDraft] = useState(formatNum(value));
  const [lastSyncedValue, setLastSyncedValue] = useState(value);

  // Reconcile draft with upstream value changes (e.g. reset, refresh) without
  // clobbering active edits. Compare against last-synced, not draft.
  if (value !== lastSyncedValue) {
    setLastSyncedValue(value);
    setDraft(formatNum(value));
  }

  const handleCommit = () => {
    if (draft === formatNum(value)) return;
    onCommit(draft);
  };

  return (
    <div className="flex flex-wrap items-center gap-3">
      <label className="flex-1 min-w-0">
        <div className="flex items-baseline justify-between gap-2">
          <span className="font-mono text-sm text-zinc-300 truncate">
            {spec.label}
          </span>
          {spec.unit && (
            <span className="text-xs text-zinc-600 font-mono">{spec.unit}</span>
          )}
        </div>
        <input
          type="number"
          step={spec.step}
          min={spec.min}
          max={spec.max}
          value={draft}
          onChange={(e) => setDraft(e.target.value)}
          onBlur={handleCommit}
          onKeyDown={(e) => {
            if (e.key === "Enter") {
              e.preventDefault();
              (e.target as HTMLInputElement).blur();
            }
          }}
          className="mt-1 w-full rounded-md border border-zinc-700 bg-zinc-950 px-2 py-1 text-sm font-mono text-zinc-100 outline-none focus:border-blue-500"
        />
      </label>
      <StatusBadge status={status} />
    </div>
  );
}

function StatusBadge({ status }: { status: Status }) {
  if (status.kind === "idle") {
    return <span className="w-6 text-zinc-700 text-center select-none">·</span>;
  }
  if (status.kind === "ok") {
    return (
      <span
        className="w-6 text-emerald-400 text-center font-bold"
        title="committed"
      >
        OK
      </span>
    );
  }
  return (
    <span
      className="max-w-[14rem] truncate text-xs text-red-400"
      title={status.msg}
    >
      {status.msg}
    </span>
  );
}

function formatNum(n: number): string {
  if (!Number.isFinite(n)) return "0";
  return String(n);
}
