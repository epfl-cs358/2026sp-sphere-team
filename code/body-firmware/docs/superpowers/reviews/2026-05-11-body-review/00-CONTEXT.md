# Body Firmware Review — Shared Context (00-CONTEXT.md)

**Review date:** 2026-05-11
**Anchored at commit:** f8ca95f (f8ca95ff7fd8117f732fb026e019d957cf209645)
**Reviewers:** 12 cell agents + 1 verifier + 1 synthesizer

## 1. Subsystem map

**motor** (`src/motor/`) — Abstract `Motor` interface (`Motor.h`) with concrete `FIT0186Motor<FilterN>` template that owns an `ESP32Encoder`, a `Driver&`, and a `MovingAverage<N>` for RPM smoothing. Exposes `begin/update/setSpeed(float [-1,1])/brake/getRPM/getFilteredRPM`. Hardware constants in `constants/FIT0186.h` (ENCODER_CPR=2800, GEAR_RATIO=43.8, NO_LOAD_RPM=251). Collaborates with: `Driver` (composition), `Drivetrain` (consumer), `MovingAverage`.

**motor/driver** (`src/motor/driver/`) — Abstract `Driver` (`Driver.h`) with two concretes: `L298NDriver` (analogWrite PWM on fwd/rev pins, brake = both HIGH) and `BTS7960Driver` (wraps `BTS7960` library, brake = `Stop()`). Both accept normalized `[-1,1]` float via `setOutput()`. `BTS7960Driver.cpp` is excluded from all current build envs.

**drivetrain** (`src/drivetrain/`) — Generic `Drivetrain<TVelocity>` interface with `drive/update(dt)/stop`. `OmniDrivetrain` is the only concrete: owns `OmniKinematics` (stateless IK math, pure functional), 3 `PID*` refs, and per-wheel `_targetRPMs[3]`. `drive()` computes targets; `update(dt)` ticks motor encoder reads, runs 3 PIDs (setpoint=target RPM, measurement=`getFilteredRPM`), writes `motor->setSpeed()`. `RobotConstants.h` centralizes geometry (WHEEL_RADIUS=0.046225m, ROBOT_RADIUS=0.1745m, TILT_ANGLE=30°, MAX_RPM=251) and loop timing (CONTROL_PERIOD_MS=10, STALENESS_TIMEOUT_MS=200).

**drivetrain/controller** (`src/drivetrain/controller/`) — Generic `DrivetrainController<TCommand,TIMUData,TVelocity>` abstract with `update(command, imu)` and `stop()`. Only concrete is `PassthroughDrivetrainController` (ignores IMU, forwards command directly to drivetrain.drive). `docs/controller-discussion.md` describes a future tilt/yaw stabilizer; not yet implemented.

**imu** (`src/imu/`) — `IMU<T>` interface returning a templated reading. `BNO055IMU` is the concrete (`IMU<IMUReading>`); wraps `Adafruit_BNO055` in `OPERATION_MODE_NDOF`, with field-mask bitset (`IMUField`) gating which sensors are read each `read()`. Reading carries `Quat`, `Euler`, `Vec3` (linearAccel/gyro/gravity), and `CalStatus`. On boot, restores sensor offsets from NVS (`Preferences` key "bno055/offsets", 22 bytes); auto-saves once when first fully calibrated.

**input** (`src/input/`) — `CommandProducer` (transport-health half) and `Lifecycle` (start/stop) split so polled producers don't need both. `MockCommandProducer<T>` is in-process injector for tests. `WebSocketCommandProducer` accepts text frames `"vx,vy,omega"` (parsed by `CommandFrameParser`), enforces single-active-client policy via `compare_exchange_strong`, pushes results into a `CommandLatch<BodyVelocity>`. Runs its own FreeRTOS task on Core 0.

**control** (`src/control/`) — `PID` with anti-windup (integral clamping against output range), derivative-on-measurement, deadband zero-snap, finite/dt asserts, returns `_lastOutput` if `dt < 1e-4`.

**util** (`src/util/`) — Plain-data types `Vec3/Quat/Euler/CalStatus`, `MovingAverage<N>` (compile-time window), `Lifecycle` interface, `debug.h` (`BB8_ASSERT` enabled by `-DBB8_DEBUG` only in `[env:native]`).

**util/sync** (`src/util/sync/`) — `CommandLatch<T>` is a FreeRTOS-queue-of-length-1 used as a single-slot mailbox (write = `xQueueOverwrite`, read = non-blocking `xQueueReceive`). `CommandLatch.cpp` is just an explicit instantiation for `BodyVelocity`.

**config** (`src/config/`) — Pins (motor + encoder GPIO maps), debug macros, and gitignored `wifi_credentials.h` (committed file currently contains live WIFI + OTA secrets, see §9).

## 2. Threading model

ESP32 has two cores; Arduino's `loop()` runs on Core 1 with the Arduino main task. `setup()`/`loop()` are in `main_robot.cpp`.

| Task | File:line | Name | Stack | Priority | Core | Job |
|------|-----------|------|-------|----------|------|-----|
| Arduino main `loop()` | n/a (framework) | loopTask | default | 1 | 1 | OTA poll + WiFi watchdog at 10 Hz (main_robot.cpp:252) |
| Control loop | `src/main_robot.cpp:240` | "control" | 8192 B | 4 | 1 | 100 Hz: latch read, staleness ramp, controller update, drivetrain PID tick, TWDT pet |
| WebSocket producer | `src/input/WebSocketCommandProducer.cpp:57` | "ws_prod" | 8192 B | 2 | 0 | Pumps `WebSocketsServer::loop()` every ~1 ms, dispatches WS events, writes latch |

Resource ownership:
- `g_drivetrain`, `g_motor*`, `g_pid*`, `g_imu`, `g_controller` — touched only by the control task after `setup()` finishes.
- `g_latch` — written by ws_prod (Core 0), read by control (Core 1). FreeRTOS queue is the cross-core safe primitive.
- `g_producer->_connected` / `_activeClient` / counters — `std::atomic` accessed from event callbacks and `controlTask` (the latter never reads them in the current code, but the API is exposed).
- WiFi/OTA state — touched only by Arduino `loopTask` (Core 1).
- Both control and ws_prod call `esp_task_wdt_add(NULL)` and reset within their loop; TWDT timeout is 1 s.

## 3. Cross-core handoff (ASCII diagram)

```
        Core 0 (NET / PRO_CPU)                          Core 1 (APP_CPU)
  +---------------------------------+         +-----------------------------------+
  | Arduino WiFi/lwIP stack         |         | Arduino main loopTask             |
  |                                 |         |   ArduinoOTA.handle()             |
  | WebSocketsServer (links2004)    |         |   WiFi.status() reboot watchdog   |
  |   |  callback (same task,       |         +-----------------------------------+
  |   v   onWsEvent)                |
  | ws_prod task (prio 2)           |         | control task (prio 4)             |
  |   parseCommandFrame()           |         |   g_latch->read()  (consumer)     |
  |   _latch.write(BodyVelocity) ---|-------->|     (FreeRTOS queue, xQueueOverwrite|
  |          (producer)             |  queue  |      / non-blocking xQueueReceive)|
  |                                 |   len=1 |   staleness ramp (last_known *    |
  |                                 |         |     (1 - age/200ms))              |
  |                                 |         |   imu.read() (I2C, blocking)      |
  |                                 |         |   controller.update(cmd, imu)     |
  |                                 |         |   drivetrain.update(dt)           |
  |                                 |         |     -> motor->setSpeed(pid_out)   |
  +---------------------------------+         +-----------------------------------+
```

Synchronization primitives by edge:
- ws_prod onWsEvent → `_latch.write` → control read: **FreeRTOS queue (len 1)** via `xQueueOverwrite` / `xQueueReceive`. Sole cross-core handoff for commands.
- ws_prod connection flags → app: **`std::atomic`** on `_connected`, `_activeClient`, `_frameCount`, `_parseFailCount`.
- ws_prod task lifetime (start/stop): **binary semaphore** `_exitSemaphore` + `std::atomic<bool> _running`; `stop()` falls back to `vTaskDelete` after 1 s timeout.
- control task → motors / IMU: single-threaded; no cross-core access after `setup()`.

## 4. Key invariants & conventions

- **Body-frame units (`BodyVelocity`)**: `vx` m/s forward, `vy` m/s left, `omega` rad/s CCW-from-above. Right-handed: x forward, y left, z up. Matches BNO055 default frame. Defined `src/drivetrain/BodyVelocity.h`.
- **Per-wheel angular velocity → RPM**: IK in `OmniKinematics::toWheelRPMs` produces rad/s then multiplies by `60/(2π)` (file:line 36). Returned array is in RPM, sign carries direction.
- **PID lives in RPM-space**: PID setpoint = target RPM, measurement = `getFilteredRPM()`, output `[-1, 1]` mapped directly to `Motor::setSpeed`. Tuning in `main_robot.cpp:66-68` and `main_drivetrain_test.cpp:22-24`: Kp=0.005, Ki=0.002, Kd=0.0005, deadband=5 RPM.
- **Wheel order / angles** (`docs/omni-kinematics-derivation.md`, asserted in IK at `OmniKinematics.h:32-34`): index 0 = front (θ=0°), index 1 = back-right (θ=120° CW), index 2 = back-left (θ=240° CW). Sin/cos baked as constants.
- **IMU axes**: BNO055 default x-forward, y-left, z-up (matches body frame). `Euler.heading` is yaw (degrees), `roll`, `pitch` per BNO055 convention. Gyro is degrees/sec (raw from `VECTOR_GYROSCOPE`). LinearAccel in m/s² (gravity-removed).
- **Calibration**: 4 channels 0–3 (sys/gyro/accel/mag). Auto-save offsets to NVS once `isFullyCalibrated()` returns true (`BNO055IMU.h:73-77`).
- **Motor command range**: `Motor::setSpeed(float)` and `Driver::setOutput(float)` accept `[-1.0, +1.0]`, asserted via `BB8_ASSERT` (debug only). `0.0` = coast on both drivers; `1.0` = forward, `-1.0` = reverse.
- **PWM range / frequency**: Both drivers use Arduino-core `analogWrite` ⇒ 8-bit (0–255) at the ESP32 Arduino default analogWrite frequency (~1 kHz LEDC). No explicit `analogWriteFrequency` or `ledcSetup` is called.
- **Brake semantics**: L298N brake = both pins HIGH (255/255 short). BTS7960 brake = `_hbridge.Stop()` (library call); BTS7960 `setOutput(0)` calls `Disable()` which differs from coast on L298N. (See §9.)
- **Coordinate frames**: body frame only; no world frame is computed (no odometry). IMU yaw is observed but currently unused by the passthrough controller.
- **Saturation**: IK proportionally scales all three wheel targets down if the max-abs exceeds `config.maxRPM` (preserves direction).
- **Staleness policy** (`main_robot.cpp:160-183`): linear ramp scale = `1 - age/200ms` for `0 ≤ age < 200ms`; hard zero beyond. Pre-first-fresh = zero.
- **Loop timing**: 100 Hz via `vTaskDelayUntil` (constant phase). Encoder dt computed from `micros()` inside `FIT0186Motor::update()` (independent of control loop's nominal dt).

## 5. Lifecycle / boot sequence (`main_robot.cpp`)

`setup()` (line 210) — strictly sequential:
1. `Serial.begin(115200)` + 200 ms `delay` (line 211-213).
2. `Wire.begin(I2C_SDA=21, I2C_SCL=22)` (line 215). Note: differs from `main_imu_test.cpp` which also uses 21/22 but the `imu-test-procedure.md` doc lists 26/25 (stale doc).
3. `g_motor{0,1,2}.begin()` — attaches ESP32Encoder, pinModes (line 217-219). H-bridge pins go OUTPUT at this point.
4. `g_imu.begin()` — blocking I2C init; **on failure** calls `ESP.restart()` (line 221-224).
5. `connectWifi()` — blocks until `WL_CONNECTED` with 30 s timeout; timeout → `ESP.restart()` (line 226).
6. Heap-allocates `g_latch`, `g_producer`, `g_controller` after FreeRTOS scheduler is up (line 228-230, intentional to avoid static-init ordering against FreeRTOS).
7. `g_producer->start()` — creates ws_prod task on Core 0 (line 232).
8. `initOta()` — registers OTA start callback that calls `g_producer->stop()` (line 234, 100-120).
9. `esp_task_wdt_init(1 s, panic=true)` (line 238).
10. `xTaskCreatePinnedToCore(controlTask, ..., core=1)` (line 240).

`loop()` (line 252) — Arduino-framework `loopTask` (Core 1, prio 1): `ArduinoOTA.handle()`, WiFi-offline reboot watchdog (30 s grace), 100 ms `vTaskDelay`. Not subscribed to TWDT.

`controlTask` (line 144) — runs `esp_task_wdt_add(NULL)`, enters infinite loop driven by `vTaskDelayUntil` at 10 ms period. On every tick: latch read → staleness compute → `imu.read()` (blocking I2C) → `controller.update()` → `drivetrain.update(dt=0.010)` → TWDT reset → optional one-shot stack HWM dump after 100 iters.

## 6. Failure / failsafe paths

- **WiFi connect timeout (30 s)** → `ESP.restart()` (`main_robot.cpp:133`).
- **WiFi offline > 30 s** (post-boot) → `ESP.restart()` (`main_robot.cpp:259-261`).
- **BNO055 begin fails** → `ESP.restart()` (`main_robot.cpp:222-223`).
- **Task watchdog timer** (`esp_task_wdt_init` with panic=true, 1 s, `main_robot.cpp:238`). Subscribed: control task (`main_robot.cpp:148`), ws_prod task (`WebSocketCommandProducer.cpp:103`). Hang → panic + reset. (IDLE tasks also affected.)
- **Command staleness** (200 ms): `main_robot.cpp:174-183`. Linear ramp to zero, then hard zero. Drives `BodyVelocity{0,0,0}` through `PassthroughDrivetrainController.update()` → `OmniDrivetrain::drive()` → kinematics → PID setpoint zero. Note: this is **not** `drivetrain.stop()`; PIDs keep running, motors don't actively brake.
- **WS half-open TCP**: `_server->enableHeartbeat(2000, 1000, 2)` (~6 s detection window, `WebSocketCommandProducer.cpp:51`). The control loop also enforces staleness, so `connected()` flag is intentionally not consulted on the hot path (`main_robot.cpp:170-171` comment).
- **OTA start callback**: `g_producer->stop()` triggers (line 109), so command flow halts; staleness ramp drives motors to zero before flash overwrite.
- **`OmniDrivetrain::stop()`** (`OmniDrivetrain.h:39-43`) brakes all motors + resets PIDs + clears `_targetRPMs`. **Never called by the runtime loop** — only invoked explicitly via `PassthroughDrivetrainController::stop()`, which itself is unused (see §9).
- **Producer destructor** (`WebSocketCommandProducer.cpp:37-40`) calls `stop()` defensively. Never invoked at runtime (statics live forever).
- **PID divide-by-zero guard**: `dt < 1e-4` returns `_lastOutput` (`PID.h:20`).

## 7. Build environments (`platformio.ini`)

| env | base | Notable src filter | Notable build_flags / extras | Libs added |
|-----|------|--------------------|------------------------------|------------|
| `wemos_d1_uno32` | (root) | excludes BTS7960Driver.cpp, FIT0186Motor.h(!), all `main_*` except none — effectively no main; default lib base | `-std=gnu++17 -Wall -Wextra` + `-I` for all src subdirs; `build_unflags=-std=gnu++11` | Adafruit BNO055 1.6.4, Adafruit Unified Sensor 1.1.15, Adafruit BusIO 1.17.4, Preferences, ArduinoLog 1.1.1, ESP32Encoder 0.12.0 |
| `drivetrain_test` | wemos_d1_uno32 | keeps `main_drivetrain_test.cpp`; excludes ws_prod, CommandLatch.cpp, other mains, BTS7960Driver.cpp | inherits | ESP32Encoder duplicated |
| `motor_test` | wemos_d1_uno32 | keeps `main_motor_test.cpp`; excludes ws_prod, CommandLatch.cpp, other mains | inherits | ESP32Encoder duplicated |
| `robot` | wemos_d1_uno32 | keeps `main_robot.cpp` + ws_prod + CommandLatch.cpp; excludes other mains and BTS7960Driver.cpp | inherits | adds links2004/WebSockets 2.7.3 |
| `robot_ota` | robot | identical sources | `upload_protocol=espota`, `upload_port=bb8-robot.local`, `--auth=${sysenv.OTA_PASSWORD}` | inherits |
| `native` | (root) | `test_build_src=false` | `-std=gnu++17 -Itest/stubs -Itest -DBB8_DEBUG` + all src `-I` | `meekrosoft/fff`, Adafruit BNO055 |

The base env's filter excludes `FIT0186Motor.h` (a header, not a unit) — harmless. None of the production envs set `-DBB8_DEBUG`; `BB8_ASSERT` is a no-op on hardware.

## 8. File map (src/ inventory)

| Path | LOC | Purpose |
|------|-----|---------|
| `src/main_blink.cpp` | 23 | GPIO smoke test (six motor pins, no logic) |
| `src/main_drivetrain_test.cpp` | 96 | Serial REPL: `<vx> <vy> <omega_deg>`, `stop`, `rpm` |
| `src/main_imu_test.cpp` | 79 | Serial REPL printing IMU fields at 10 Hz |
| `src/main_motor_test.cpp` | 89 | Serial REPL: `<motor> <speed>`, encoder RPM |
| `src/main_original.cpp` | 56 | Earlier 3-motor spin test (no encoders) |
| `src/main_robot.cpp` | 264 | Production: WS teleop + 100 Hz control |
| `src/config/debug.h` | 27 | `BB8_ASSERT` (only firing in `[env:native]`) |
| `src/config/pins.h` | 22 | GPIO + encoder pin definitions |
| `src/config/wifi_credentials.example.h` | 19 | Template for the (gitignored?) credentials |
| `src/config/wifi_credentials.h` | 10 | Live credentials + OTA password (see §9) |
| `src/control/PID.h` | 74 | PID with anti-windup, deadband, dt guard |
| `src/drivetrain/BodyVelocity.h` | 7 | `{vx, vy, omega}` POD |
| `src/drivetrain/Drivetrain.h` | 29 | Generic interface `drive/update/stop` |
| `src/drivetrain/DrivetrainConfig.h` | 8 | Geometry POD |
| `src/drivetrain/OmniDrivetrain.h` | 55 | 3-omniwheel impl, owns 3 PIDs |
| `src/drivetrain/OmniKinematics.h` | 57 | Pure IK + saturation scaling |
| `src/drivetrain/RobotConstants.h` | 27 | Geometry + loop timing constants |
| `src/drivetrain/controller/DrivetrainController.h` | 28 | Generic controller interface |
| `src/drivetrain/controller/PassthroughDrivetrainController.h` | 24 | Forwards command, ignores IMU |
| `src/imu/BNO055IMU.h` | 116 | `IMU<IMUReading>` over Adafruit_BNO055 + NVS calib |
| `src/imu/IMU.h` | 16 | Generic IMU interface |
| `src/imu/IMUReading.h` | 38 | `IMUField` mask + `IMUReading` POD |
| `src/input/CommandFrameParser.h` | 59 | `"vx,vy,omega"` text parser (header-only) |
| `src/input/CommandProducer.h` | 18 | `connected()` half of producer interface |
| `src/input/MockCommandProducer.h` | 31 | In-process injector for tests |
| `src/input/WebSocketCommandProducer.h` | 69 | WS+Lifecycle producer (forward decl style) |
| `src/input/WebSocketCommandProducer.cpp` | 187 | WS task body, event handling, single-client policy |
| `src/motor/FIT0186Motor.h` | 82 | Encoder + filter + driver wrapper |
| `src/motor/Motor.h` | 28 | Generic motor interface |
| `src/motor/MotorConfig.h` | 18 | `EncoderPins`, `MotorConfig` PODs |
| `src/motor/constants/FIT0186.h` | 14 | Spec constants for FIT0186 |
| `src/motor/driver/BTS7960Driver.cpp` | 40 | Wraps BTS7960 lib (excluded from all envs) |
| `src/motor/driver/BTS7960Driver.h` | 29 | (header still included by envs?) |
| `src/motor/driver/Driver.h` | 19 | Generic H-bridge interface |
| `src/motor/driver/L298NDriver.cpp` | 42 | analogWrite-based bidirectional driver |
| `src/motor/driver/L298NDriver.h` | 26 | Header for L298N driver |
| `src/util/CalStatus.h` | 15 | BNO055 calibration POD |
| `src/util/Euler.h` | 12 | `{heading, roll, pitch}` POD |
| `src/util/Lifecycle.h` | 24 | `start/stop` interface |
| `src/util/MovingAverage.h` | 41 | Compile-time-sized ring average |
| `src/util/Quat.h` | 13 | `{w,x,y,z}` POD |
| `src/util/Vec3.h` | 12 | `{x,y,z}` POD |
| `src/util/sync/CommandLatch.cpp` | 11 | Explicit `template class CommandLatch<BodyVelocity>` |
| `src/util/sync/CommandLatch.h` | 39 | Queue-of-1 mailbox with `std::optional` read |

## 9. Known gotchas & open questions (observations only)

1. `src/config/wifi_credentials.h` is checked in (visible in git ls-files and the file's own header says "gitignored. Do NOT commit."), containing what appears to be a real SSID/password and OTA password `"bb8"`. The `.example.h` sibling exists too. Observation: credentials live in the tree at this SHA.
2. `BTS7960Driver` and `L298NDriver` diverge at `setOutput(0)`: L298N writes 0,0 (coast). BTS7960 calls `_hbridge.Disable()` (also coast, but via different code path); BTS7960's brake calls `_hbridge.Stop()`, L298N's brake writes 255/255. The two `Driver`s also disagree about whether to re-`Enable` on every non-zero call (BTS7960 does).
3. `BTS7960Driver.cpp` is excluded from every build env's `build_src_filter`, but the header is reachable via includes; it currently has no callers in `src/`.
4. `IMUField operator|` returns `IMUField`, but `operator&` returns `bool` (`IMUReading.h:23-29`). Asymmetric — composing more than two masks with `&` won't work.
5. `OmniKinematics::toWheelRPMs` reconstructs `cosAlpha`, `scale`, and the saturation scan every call (no caching across calls). Called at 100 Hz from `OmniDrivetrain::drive()` only on new commands, but `update()` does not call `drive()` again, so this cost is per-command, not per-tick.
6. `FIT0186Motor::update()` computes its own dt from `micros()` regardless of the caller's dt. `OmniDrivetrain::update(dt)` then passes the caller's dt to the PID. The two dt values are independently sourced; under normal scheduling they agree to within a few µs.
7. `MovingAverage<5>` is constructed per `FIT0186Motor` instance and is touched only from the control task (Core 1). It is templated but not used cross-core in the current code.
8. `CommandLatch<T>` is templated; `CommandLatch.cpp` only instantiates `<BodyVelocity>`. In `[env:drivetrain_test]` and `[env:motor_test]` the cpp is excluded from the filter — fine because nothing in those envs uses `CommandLatch`.
9. The control task's staleness ramp scales `last_known` linearly, applying the same scale to `omega` as to `vx/vy`. There is no distinction between translation timeout and rotation timeout.
10. The control loop calls `g_imu.read()` every tick (10 ms) even though the passthrough controller ignores it. The BNO055 `read()` performs up to six I2C transactions depending on field mask (here: Quat+Euler+Gyro+Cal = 4).
11. `PassthroughDrivetrainController::stop()` exists but is never wired to any runtime signal. WS disconnect, stale command, and OTA all end up zeroing the command rather than calling `stop()`. Net effect at zero command is PIDs running with setpoint=0, motors driven via PID output, not active brake.
12. Pin docs vs reality: `docs/motor-spin-test-procedure.md` lists motor 2 at GPIO 18/19 and encoder 1 at A=4/B=2; `src/config/pins.h` has motor 2 at 25/26 and encoder 1 at A=2/B=4. The most recent commit (f8ca95f "remap motor and IMU pins to match new harness") moved the source; docs lag.
13. `[env:wemos_d1_uno32]` (the inherited base) excludes every `main_*.cpp`. Building this env alone wouldn't link (no `setup`/`loop`). Existence is only as a base for `extends =` envs.
14. `IMUReading::orientation` defaults to identity `Quat{1,0,0,0}` (via `Quat`'s defaults), but `linearAccel/gyro/gravity` default to zero. Field-mask gating means unread fields keep their defaults.
15. `OmniDrivetrain::drive()` is called from `controlTask` only via the controller's `update()`, which calls it on every tick (including when the command hasn't changed). The kinematics recompute is intentional but doubles as an opportunity to memoize.
16. WS protocol is unauthenticated `ws://`, single client gated by `compare_exchange_strong` on `_activeClient`. A second connecting client is `disconnect()`ed by the server but may be able to spam connection attempts. No rate limit on `WStype_CONNECTED`.
17. `WiFi.setAutoReconnect(true)` is set, and `loop()` only restarts after 30 s. The control loop does not depend on WiFi (commands come from the latch, which retains the last value indefinitely until the staleness window expires).
18. The OTA start callback calls `g_producer->stop()`, which is **blocking up to 1 s** waiting on `_exitSemaphore`. This runs inside `ArduinoOTA.handle()` on the Arduino loopTask (Core 1). Control task is unaffected because it runs on Core 1 with higher priority.
19. `_targetRPMs` in `OmniDrivetrain` is non-atomic but only touched from the control task — currently safe.
20. The `[env:robot]` filter still excludes `BTS7960Driver.cpp`. Switching to BTS7960 hardware requires editing both the env filter and `main_robot.cpp` driver-instantiation lines (currently `L298NDriver`).

## 10. Glossary

- **BodyVelocity** — POD `{vx, vy, omega}` in body frame. The only command type that crosses the WS→control boundary.
- **OmniKinematics** — Stateless functor; maps `BodyVelocity` → 3-element RPM array with tilt compensation `1/cos(α)` and proportional saturation.
- **OmniDrivetrain** — Concrete `Drivetrain<BodyVelocity>`; owns motors+PIDs+kinematics. `drive()` updates targets; `update(dt)` ticks the control loop.
- **CommandLatch<T>** — Single-slot mailbox over a FreeRTOS queue of length 1. Write overwrites; read is non-blocking and returns `std::optional<T>`.
- **CommandProducer** — Interface exposing `connected()`. Paired with `Lifecycle` (start/stop) only when the producer owns a task.
- **Lifecycle** — Opt-in `start()/stop()` interface for FreeRTOS-task-owning components.
- **CalStatus** — BNO055 4-channel calibration `{sys, gyro, accel, mag}`, each 0–3.
- **IMUField** — Bitset selecting which BNO055 vectors `read()` will fetch.
- **IMUReading** — POD bundle of (optionally populated) IMU outputs.
- **PassthroughDrivetrainController** — Trivial `DrivetrainController` that forwards command to drivetrain and discards IMU.
- **STALENESS_TIMEOUT_MS** — 200 ms; linear ramp window before command goes to zero.
- **CONTROL_PERIOD_MS** — 10 ms; control task tick.
- **TWDT** — ESP-IDF Task Watchdog Timer; 1 s panic timeout in production.
- **Active client** — The single WS client allowed to drive at any time; subsequent connects are disconnected by the server.
- **Filtered RPM** — `MovingAverage<5>` over raw RPM samples, fed to PID measurement input.
- **Saturation scaling** — IK shrinks all 3 wheel commands by a common factor when max exceeds `maxRPM`, preserving direction.
