# BB-8 tuning tools

`tune_pid.py` is a persistent telemetry capture daemon plus a set of LLM-safe inspection commands for tuning the balancing PID. The `tuning-pid-balance` Claude skill (`~/.claude/skills/tuning-pid-balance/SKILL.md`) drives this tool.

## Quick start (foreground)

Install deps once, then run the daemon in the background:

```
pip install -r tools/requirements.txt
nohup python3 tools/tune_pid.py capture --host bb8.local > /tmp/cap.log 2>&1 &
```

Capture writes to `~/.bb8-telemetry/sessions/<id>/` and updates the `~/.bb8-telemetry/current` symlink. Ctrl-C / `kill` flushes cleanly. Re-launching starts a new session.

## Quick start (launchd)

Persistent across reboots and login sessions. Edit `tools/com.bb8.tune-capture.plist` and replace `<USERNAME>` with your macOS username and `<HOST>` with the robot's mDNS name (default `bb8.local`), then:

```
cp tools/com.bb8.tune-capture.plist ~/Library/LaunchAgents/com.bb8.tune-capture.plist
launchctl load   ~/Library/LaunchAgents/com.bb8.tune-capture.plist
launchctl list | grep com.bb8.tune-capture          # verify
```

Logs land in `/tmp/bb8-tune-capture.{out,err}.log`. To stop:

```
launchctl unload ~/Library/LaunchAgents/com.bb8.tune-capture.plist
```

## Install the tuning skill

The skill is already in place at `~/.claude/skills/tuning-pid-balance/SKILL.md`. Invoke from any Claude session via `/tuning-pid-balance`.

## Inspection commands cheat sheet

All subcommands operate on `--session current` by default. Output is bounded JSON — safe to pipe into Claude via Bash.

| Command | Purpose |
|---|---|
| `tune_pid.py sessions` | List sessions with start time, end time, and row count. |
| `tune_pid.py manifest` | Print the active session's manifest (gain + arming history, fault count, seq gaps). |
| `tune_pid.py summary --window 10s` | Default read. Constant-size JSON: per-axis min/max/mean/std, event counts, saturation %, oscillation period if any. |
| `tune_pid.py events --window 30s` | Decoded event rows (`event_flags != 0`) only. Capped at 200 rows. |
| `tune_pid.py slice --around <seq> --max-rows 200 --columns ...` | Narrow window around an event, downsampled, named columns only. |
| `tune_pid.py oscillation --axis pitch --window 10s` | Autocorrelation period + amplitude. Used to derive Tu in Ziegler-Nichols. |
| `tune_pid.py plot --window 30s --out /tmp/last.png` | Six-panel PNG with event markers. Read via Claude's multimodal image input. |
