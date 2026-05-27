#  Sphere Robot

<img width="326" height="500" alt="WhatsApp Image 2026-05-27 at 22 32 17" src="https://github.com/user-attachments/assets/235ac963-14b6-4ab2-a806-31db804c82da" />

A self-contained spherical robot inspired by BB-8. CS-358 final project, EPFL.

---

## Project Overview

It is a self-contained spherical robot inspired by BB-8 from Star Wars. A 3D-printed shell rolls on the ground, driven from the inside by a small chassis with three omnidirectional wheels. On top of the sphere, a separate "head" is held only by magnets, so it can rotate freely as the sphere moves. The head carries a camera.

The robot:

- Rolls in any direction using 3 omni wheels at 120° spacing, tilted inward at 30° against the inside of the sphere.
- Is driven over Wi-Fi from a browser dashboard.
- Streams live video from the head camera (planned — head firmware is in progress).
- Self-stabilizes with a 9-DOF IMU and a 100 Hz control loop.
- Can be reflashed over Wi-Fi (OTA), so the sphere doesn't have to be opened.

---

## Table of Contents

- [Bill of Materials](#bill-of-materials)
- [Build the Robot](#build-the-robot)
- [Algorithms](#algorithms)
- [Control Architecture](#control-architecture)
- [Communication Protocol](#communication-protocol)
- [Web App](#web-app)
- [Code — Repository Structure](#code--repository-structure)
- [Quick Start](#quick-start)
- [Make Commands](#make-commands)
- [Body Firmware (ESP32 C++)](#body-firmware-esp32-c)
- [Head Firmware](#head-firmware)
- [Webapp (Next.js — port 3003)](#webapp-nextjs--port-3003)
- [Stream Client + Relay (ports 3004/3005)](#stream-client-nextjs--port-3004--relay-port-3005)
- [System Requirements](#system-requirements)
- [WiFi Configuration](#wifi-configuration)

---

## Bill of Materials

### Electronic Components

| Component | Part | Notes | Link |
|---|---|---|---|
| Microcontroller (body) | ESP32 Wemos D1 R32 / UNO32 | Runs the control loop and the WebSocket server | Course provided |
| Camera (head) | XIAO Vision AI Camera (Seeed Studio) | Streams video from the head | [Digikey](https://www.digikey.ch/de/products/detail/seeed-technology-co-ltd/102010635/26553888?gclsrc=aw.ds&gad_source=1&gad_campaignid=23648664371&gbraid=0AAAAADrbLlgfsbqUJlMFRLgHC9VhafRgX&gclid=Cj0KCQjwm6POBhCrARIsAIG58CKoRNC8lP7SramCpo4bx8WypJQ6x6bo5LUtMgXISv5xn2gV4Q7les8aAh8cEALw_wcB) |
| Motors (×3) | DFRobot FIT0186 | 12 V DC gear motors with encoders, one per omni wheel | [Bastel Garage](https://www.bastelgarage.ch/moteur-a-engrenages-dc-12v-251rpm-18kg-cm-avec-encodeur?search=Moteur%20DC%2012V%20251RPM%20) |
| Motor drivers (×3) | L298N H-bridge modules | One channel per motor | Course provided |
| IMU | Adafruit BNO055 | 9-DOF with built-in sensor fusion, I²C address 0x28 | [Bastel Garage](https://www.bastelgarage.ch/bno055-intelligent-9-axis-sensor) |
| Battery | LiPo | 5000mAh / 3S = 11V | Course provided |
| BMS | [TODO: model] | Protects the LiPo | Course provided |
| Buck converter | LM2596 (or equivalent) | 12 V → 5 V for the ESP32 | Course provided |
| Magnets (x6) | Neodymium | (Ø 20 mm, height 5 mm N42) ; hold the head onto the sphere | [Supermagnete](https://www.supermagnete.ch/fre/aimants-disques-neodyme/disque-magnetique-20mm-5mm_S-20-05-N)|

!! ball bearing || SERVO 


### Mechanical Parts (Body)

| Part | Material | Notes | File |
|---|---|---| --- |
| Omni wheels (×3), ~9.25 cm diameter | PETG/TPU  (3D printed) | | |
| Metal sticks (x15), 3mmx35mm | Steel (any metal can be used) |  Metals parts used to hold the omni wheels assembly |  |
| Motor Holder | PETG (3D printed) | One peice that holds the 3 motors at 120°, tilted inward by 30° | [Motor Holder](https://github.com/./README_files/Motor holder.step) |
| Internal plates | MDF (laser-cut) | Structure for the electronical components |  |
| BMS Box | PETG (3D printed)  |  |  |
| Tube Holder | PETG (3D printed) | Holds the tube for the magnets |  
| 2 metal pipes (1.5mm(external diamateter) x 60mm & 2.5mm(external diamateter) x 200mm)| Aluminium (any metal can be used) | Tubes to hold and adjust the magnet holder and upper ball bearing |  |
| Hose clamp | PETG (3D printed) | Made to hold the bard in the body and make the magnets height adjustable |  |
| Magnet Holder | PETG (3D printed) | Holds all 3 magnets at the correct place |  |
| Magnet Holder | PETG (3D printed) | Holds all 3 magnets at the correct place |  |
| Sphere shell, 35 cm diameter | PETG (3D printed) | Two hemispheres, joined with magnets and tape |  |


### Mechanical Parts (Head)

| Part | Material | Notes | File 
|---|---|---| --- |
| Head-to-sphere contact | Metal ball-bearings in 3D-printed cages | Low-friction contact between head and sphere |  |
| Balls (14mm diameter) | Stainless Steel | Balls used in the heads ball bearing |  |
| Head shell | PETG (3D printed) | Holds the camera and magnets |  |


---

## Build the Robot

### Sphere Shell

The sphere is what the wheels push against, so it needs to be round, rigid, and smooth on the inside. Inner radius is ~17.5 cm (35 cm diameter), matching the `ROBOT_RADIUS` constant in the firmware.

1. 3D print the two hemispheres (top and bottom) in PETG on a large-format printer (e.g. Prusa XL).
2. [TODO: slicer settings — layer height, walls, infill, supports]
3. Sand the inside of each hemisphere until it feels smooth. This is what gives the wheels good grip.
4. Only join the two hemispheres at the very end, once the chassis is in place inside.
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/adb60fc4-800e-4eca-8d98-96ad8852bf2b" />
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/286072ff-c11b-4bee-bda8-11f4af2c0cca" />

### Internal Chassis

The chassis holds the three motors at 120° spacing, each tilted inward at 30° from vertical so the wheels press against the inside of the sphere.

1. Laser-cut the chassis pieces from MDF.
2. Assemble with screws. Test-fit the three motors first — don't glue or screw anything down until they fit cleanly.
3. Mount the motors at 0°, 120°, and 240° looking from above (front, back-right, back-left).
4. Mount the IMU as far as possible from the head magnets, and as low as possible in the chassis. A low center of mass keeps the robot stable.
   
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/5e29a8db-70be-4f34-a20c-36a505c2a91f" />

### Motors and Wheels

1. Print the 3 omni-wheel assemblies in PETG. Check that the sub-rollers spin freely.
2. Press-fit each wheel onto the motor's D-shaft.
3. Check the rolling direction. Motor 0 is at the front, motor 1 back-right, motor 2 back-left. If a motor spins the wrong way, swap fwd/rev in `body-firmware/src/config/pins.h` rather than rewiring.

### Electronics Assembly

**Electronics schematics**
<img width="450" height="599" alt="image" src="https://github.com/user-attachments/assets/6c987e40-59d6-41f3-8db5-93d1d7340a63" />


**Motor drivers (L298N)**

| Motor | ESP32 GPIO (forward) | ESP32 GPIO (reverse) |
|---|---|---|
| 0 (front) | 14 | 27 |
| 1 (back-right) | 16 | 17 |
| 2 (back-left) | 25 | 26 |

Jumper EN HIGH on all three L298N boards. Connect L298N GND to ESP32 GND.

**Encoders**

| Motor | Encoder A | Encoder B |
|---|---|---|
| 0 | 34 | 35 |
| 1 | 2 | 4 |
| 2 | 36 | 39 |

**IMU (BNO055)**

| BNO055 | ESP32 |
|---|---|
| VIN | 3.3 V |
| GND | GND |
| SDA | 21 |
| SCL | 22 |

I²C address: 0x28.

**Power wiring**

```
LiPo ──┬── BMS ──┬── 12 V ──── L298N supply (×3)
       │         │
       │         └── LM2596 ── 5 V ── ESP32
       │                            └── BNO055 + encoders (via ESP32 3.3 V)
       └── status LED
```
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/1c49d482-4739-4087-a9eb-67f866b7c235" />
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/8a8ec03a-3ec9-492f-8c21-2e44acd8e312" />


> Add bulk capacitance (≥ 1000 µF + 0.1 µF ceramic) on the 12 V rail near each L298N. Without it, the motors will brown out the ESP32 every time you accelerate.

### Head and Camera

The head houses the XIAO Vision AI Camera and its own small LiPo. It joins the same Wi-Fi as the body but streams video separately.

1. 3D print the head shell.
2. Mount the camera with the lens facing forward.
3. Wire the head LiPo through its own BMS and buck converter to the camera's 5 V input.

[TODO: photo of the head opened]

### Magnetic Head Mount

The head holds onto the sphere with magnets only — there is no mechanical link. That's what lets the head swivel independently while the sphere rolls.

1. Press neodymium magnets into the head's underside, alternating polarities.
2. Press matching magnets onto the top of the chassis (which sits inside the sphere, just under the head).
3. Metal ball-bearings in 3D-printed cages provide low-friction contact between the head and the top of the sphere.
<img width="285" height="379" alt="image" src="https://github.com/user-attachments/assets/ad9a2684-88a6-45ed-ae72-f0e17b670557" />

---

## Algorithms

### Omni-Wheel Inverse Kinematics

Given a desired body velocity (vx, vy, ω), we compute each wheel's target speed using the geometry of a 120°-spaced, inward-tilted omni layout.

For a wheel at angle θᵢ from the front:

```
ωᵢ = (1 / (r · cos α)) · [ −sin(θᵢ)·vx − cos(θᵢ)·vy − R·ω ]
```

where `r` is the wheel radius, `R` is the distance from the chassis center to the wheel contact, and `α` is the wheel tilt.

Robot constants (from `RobotConstants.h`):

| Parameter | Value |
|---|---|
| Wheel radius `r` | 0.046225 m (9.245 cm diameter) |
| Robot radius `R` | 0.1745 m (center to wheel contact) |
| Tilt angle `α` | 30° |
| Max RPM | 251 (per-wheel saturation) |

For our layout (θ₀ = 0°, θ₁ = 120°, θ₂ = 240°):

```
           1            ┌  0     −1     −R  ┐   ┌ vx ┐
[ωᵢ] = ───────────  ·   │ −√3/2   1/2  −R   │ · │ vy │
       r · cos(α)       └  √3/2   1/2  −R   ┘   └  ω ┘
```

Quick sanity checks:
- Pure forward → motor 0 doesn't spin, motors 1 and 2 spin opposite ways. ✓
- Pure rotation → all three spin in the same direction at the same speed. ✓
- Flat layout (α = 0) → equations match the standard 3-omni textbook case. ✓

Full derivation: `code/body-firmware/docs/omni-kinematics-derivation.md`

### Per-Wheel PID

Each motor has its own PID loop tracking the target speed from the kinematics.

| Parameter | Value |
|---|---|
| Input | Target RPM from kinematics |
| Feedback | Actual RPM from encoder |
| Output | PWM duty cycle (−1.0 to +1.0) |
| Kp / Ki / Kd | 0.005 / 0.002 / 0.0005 |
| Integrator clamp | ±5.0 |
| Loop rate | 100 Hz |

Code: `code/body-firmware/src/control/PID.h`

### Tilt-Limit Controller

The motors are powerful enough that on hard acceleration the chassis can climb the inside of the sphere and flip over. To prevent this, a controller watches the IMU:

| State | Condition | Action |
|---|---|---|
| Active | tilt < 15° | Normal driving |
| Recovery | tilt 15–20° | Coast motors, ignore joystick, wait for gravity. Re-engages once tilt drops ~3° |
| Stopped | tilt > 20° | Brake all motors, reset PIDs |
| Hard cap | tilt > 30° | Brake immediately, regardless of mode |

> **Status**: designed (see `controller-discussion.md`) but not yet integrated. The current shipping controller is a simple pass-through (`PassthroughDrivetrainController`), which is fine on flat ground with a careful operator.

---

## Control Architecture

### How the Two ESP32 Cores Split the Work

- **Core 0** runs the WebSocket server. It receives `vx,vy,omega` text frames from the laptop and drops the latest one into a one-slot mailbox (old commands are overwritten, never queued).
- **Core 1** runs the 100 Hz control loop. It reads the latest command, reads the IMU, runs the kinematics and the PIDs, and updates the motors.

```
WebSocket frame → mailbox → control loop (100 Hz) → kinematics → 3 × PID → 3 × motor
```

### Safety Properties

- **Stale commands stop the robot.** If no fresh command arrives within 200 ms, motors ramp smoothly to zero. No jerk, no runaway.
- **Watchdog reboot.** If the control loop ever hangs, the ESP32 resets itself within 1 second.
- **Wi-Fi watchdog.** If Wi-Fi has been disconnected for more than 30 s, the ESP32 reboots to reconnect cleanly.
- **Safe OTA.** Flashing over Wi-Fi first stops accepting commands, so the ramp-to-zero takes effect before the firmware is overwritten.
- **Latest command wins.** A joystick command from a second ago is worse than no command, so we never queue — we always use the most recent frame.

---

## Communication Protocol

The laptop talks to the body ESP32 over WebSocket on the local Wi-Fi.

- The ESP32 is the server, listening on port 80.
- The webapp is the client.

### Command Format

Plain text, comma-separated:

```
"vx,vy,omega"
```

| Field | Unit | Meaning |
|---|---|---|
| `vx` | m/s | Forward (positive = forward) |
| `vy` | m/s | Strafe (positive = left) |
| `omega` | rad/s | Yaw rate (positive = counterclockwise from above) |

Extra fields at the end are ignored — that's how we'll add new fields later (e.g. `head_tilt`) without breaking older clients.

### Video Stream

The head camera streams video to a small relay server (`code/stream-client/relay.ts`) running on the laptop. The webapp subscribes to the relay to display the feed.

- Camera → `ws://<laptop-ip>:3005/?role=producer`
- Browser → `ws://<laptop-ip>:3005/`

The relay fans out every frame from the camera to every viewer.

---

## Web App

[TODO: screenshot of the webapp dashboard]

What you can do from the dashboard:

- **Live camera feed** — real-time video from the head.
- **Camera status** — online / offline / reconnecting.
- **Manual control** — joystick or keyboard input.
- **Emergency stop** — immediately sends `0,0,0` and locks input until you resume.
- **Telemetry** — IMU pose, battery voltage, loop rate.

The webapp is a Next.js app on port 3003. The video relay is a small Node server on port 3005. The video viewer is another Next.js app on port 3004.

Before running it, set your Wi-Fi credentials in `code/body-firmware/src/config/wifi_credentials.h`.

---

## Code — Repository Structure

```
code/
├── body-firmware/          # ESP32 C++ firmware for the body (motors, IMU, teleop)
│   ├── src/
│   │   ├── config/         # Pin definitions, WiFi credentials, debug macros
│   │   ├── control/        # Custom PID controller
│   │   ├── drivetrain/     # Omni kinematics, OmniDrivetrain, RobotConstants
│   │   │   └── controller/ # PassthroughDrivetrainController
│   │   ├── imu/            # BNO055 driver (orientation, gyro, calibration)
│   │   ├── input/          # WebSocketCommandProducer, CommandFrameParser
│   │   ├── motor/          # L298N, FIT0186Motor drivers, encoders
│   │   │   ├── constants/  # FIT0186-specific motor constants
│   │   │   └── driver/     # L298NDriver, BTS7960Driver
│   │   └── util/           # Shared types (Vec3, Quat, Euler, MovingAverage…)
│   │       └── sync/       # CommandLatch (thread-safe command queue)
│   ├── test/               # PlatformIO unit tests (native + hardware)
│   │   ├── test_omni_kinematics/
│   │   ├── test_omni_drivetrain/
│   │   ├── test_pid/
│   │   ├── test_command_latch/
│   │   ├── test_command_frame_parser/
│   │   ├── test_bno055_imu/
│   │   ├── test_fit0186_motor/
│   │   ├── test_l298n_driver/
│   │   ├── test_passthrough_controller/
│   │   ├── test_moving_average/
│   │   └── stubs/          # Arduino stubs for native tests
│   ├── platformio.ini
│   └── LIBRARIES.md
├── head-firmware/          # ESP32 firmware for the head (camera/streaming)
│   └── src/config/
├── webapp/                 # Next.js control interface (port 3003)
│   └── src/
│       ├── app/            # Next.js pages (home, /video)
│       ├── components/     # React components (video-stream.tsx)
│       └── lib/video/      # VideoSource abstraction
├── stream-client/          # Next.js streaming client (port 3004)
│   ├── src/app/
│   └── relay.ts            # WebSocket relay server (port 3005)
├── Makefile                # Unified commands for the whole monorepo
├── package.json
└── pnpm-workspace.yaml
```

---

## Quick Start

```bash
make setup   # Install all dependencies
make dev     # Start all services
```

Once running:
- Webapp control interface[TODO : link] 
- Stream client: [TODO : link] 
- WebSocket relay: [TODO : link] 

---

## Make Commands

### Development

| Command | Description |
|---|---|
| `make dev` | Start all services (webapp + stream + relay) |
| `make dev-webapp` | Webapp only (port 3003) |
| `make dev-stream` | Stream client only (port 3004) |
| `make dev-relay` | Relay server only (port 3005) |
| `make dev-stop` | Stop all dev servers (ports 3003–3005) |

### Testing

| Command | Description |
|---|---|
| `make test` | Run all tests |
| `make test-firmware` | Native PlatformIO tests (no hardware required) |
| `make test-firmware-head` | PlatformIO tests on hardware |
| `make test-webapp` | Webapp Vitest suite |
| `make test-stream` | Stream client Vitest suite |

### Build & Flash

| Command | Description |
|---|---|
| `make build` | Build entire project |
| `make build-webapp` | Build the webapp |
| `make build-stream` | Build the stream client |
| `make build-firmware` | Compile firmware (no upload) |
| `make flash` | Compile and flash firmware to the robot |
| `make monitor` | Serial monitor (115200 baud) |

### Code Quality

| Command | Description |
|---|---|
| `make lint` | ESLint on both web projects |
| `make typecheck` | TypeScript type checking |

### Setup

| Command | Description |
|---|---|
| `make setup` | Full setup (prerequisites + web + firmware) |
| `make setup-prereqs` | Check/install system dependencies |
| `make setup-web` | Install pnpm dependencies |
| `make setup-firmware` | Install PlatformIO + firmware libraries |
| `make clean` | Remove all build artifacts |

---

## Body Firmware (ESP32 C++)

### Teleop Pipeline Architecture

```
WebSocket client ──text frame──▶ WebSocketCommandProducer (Core 0)
                                          │
                                          ▼
                                  CommandLatch (latest-wins)
                                          │
                                          ▼
                              controlTask @ 100 Hz (Core 1)
                                          │
                                          ▼
                          PassthroughDrivetrainController
                                          │
                                          ▼
                                   OmniDrivetrain
                                  (3× FIT0186Motor + PID)
```

Commands are received over WebSocket on Core 0, stored in a `CommandLatch` (latest-wins, thread-safe), and consumed by the control task running at 100 Hz on Core 1. This dual-core split prevents WebSocket handling from interfering with motor control timing.

### Command Format

Commands are sent as plain text over WebSocket:

```
"vx,vy,omega"
```

| Field | Unit | Description |
|---|---|---|
| `vx` | m/s | Forward velocity |
| `vy` | m/s | Lateral (left) velocity |
| `omega` | rad/s | Rotation rate |

Frame convention: X-forward, Y-left, Z-up (consistent with BNO055 in NDOF mode).

Example: `"0.5,0.0,0.3"` moves the robot forward at 0.5 m/s while rotating at 0.3 rad/s.

### Safety Features

- **Stale command ramp-to-zero**: if no command is received within 200 ms, motors ramp down to zero.
- **Task watchdog**: 1-second timeout on both the control task and WebSocket task.
- **WiFi auto-reboot**: if the robot has been offline for 30 seconds, it automatically reboots.
- **No authentication**: WebSocket runs on `ws://` port 80 — intended for use on a trusted local network only.

### 3-Wheel Omni Kinematics

`OmniKinematics` converts a body-frame velocity `(vx, vy, omega)` into target RPM for each of the 3 motors arranged at 120° intervals. If any motor exceeds `maxRPM`, all outputs are normalized proportionally.

| Parameter | Value |
|---|---|
| Wheel radius | 46.225 mm |
| Robot radius | 174.5 mm |
| Tilt angle | 30° |
| Max RPM | 251 RPM |
| Control period | 10 ms (100 Hz) |
| Stale window | 200 ms |

### PID Controller

Custom PID with:
- **Anti-windup** via integral clamping
- **Derivative on measurement** (not on error) to avoid setpoint-change spikes
- **Configurable deadband**
- Output range: `[-1.0, 1.0]`

Current parameters: `Kp=0.005`, `Ki=0.002`, `Kd=0.0005`

### IMU — BNO055

- Mode: NDOF (9-DOF sensor fusion)
- Outputs: orientation quaternion, Euler angles, gyroscope, linear acceleration, gravity vector, calibration status
- Calibration offsets are saved to and restored from NVS flash (`Preferences`)

### Pin Assignment

| Signal | ESP32 Pin |
|---|---|
| Motor 0 FWD | 14 |
| Motor 0 REV | 27 |
| Motor 1 FWD | 16 |
| Motor 1 REV | 17 |
| Motor 2 FWD | 25 |
| Motor 2 REV | 26 |
| Encoder 0 A/B | 34 / 35 |
| Encoder 1 A/B | 2 / 4 |
| Encoder 2 A/B | 36 / 39 |
| IMU SDA | 21 |
| IMU SCL | 22 |

### PlatformIO Environments

| Environment | Purpose |
|---|---|
| `robot` | USB flash — full teleop firmware |
| `robot_ota` | OTA flash over WiFi (ArduinoOTA) |
| `drivetrain_test` | Drivetrain test firmware with RPM display |
| `motor_test` | Motor + encoder test firmware |
| `wemos_d1_uno32` | Base environment (IMU only) |
| `native` | Unit tests on the host machine |

### OTA Updates

1. Copy `wifi_credentials.example.h` → `wifi_credentials.h` and fill in your SSID, password, and OTA password.
2. Export the OTA password: `export OTA_PASSWORD='your-password'`
3. Flash: `pio run -e robot_ota -t upload`
4. If mDNS resolution fails: `pio run -e robot_ota -t upload --upload-port=<robot-ip>`

> During an OTA update, the `WebSocketCommandProducer` is stopped before flash write — motors stop within the stale window (≤ 200 ms).

### Unit Tests

Tests are located in `body-firmware/test/` and cover:

- Omni kinematics (`test_omni_kinematics`, `test_omni_drivetrain`)
- PID controller (`test_pid`)
- Command synchronization (`test_command_latch`, `test_command_frame_parser`)
- Motor drivers (`test_l298n_driver`, `test_fit0186_motor`, `test_bts7960_driver`)
- Passthrough controller (`test_passthrough_controller`)
- Utilities (`test_moving_average`)
- IMU (`test_bno055_imu`)

Run native tests (no hardware needed):
```bash
pio test -e native
```

Run tests on hardware:
```bash
make test-firmware-head
```

### Firmware Libraries

| Library | Version | Role |
|---|---|---|
| Adafruit BNO055 | 1.6.4 | IMU driver |
| Adafruit Unified Sensor | 1.1.15 | Sensor abstraction layer |
| Adafruit BusIO | 1.17.4 | I2C/SPI communication |
| ESP32Encoder | 0.12.0 | Hardware quadrature encoder reading |
| WebSockets (links2004) | 2.7.3 | WebSocket server for command reception |
| ArduinoOTA | built-in | OTA firmware updates over WiFi |
| FreeRTOS | built-in | Dual-core task scheduling |
| FFF (meekrosoft) | latest | Mocking framework for unit tests |

---

## Head Firmware

The head firmware runs on a second ESP32 and handles camera capture and video streaming. It connects to the stream relay over WebSocket and continuously sends JPEG frames.

Configuration is located in `head-firmware/src/config/`.

---

## Webapp (Next.js — port 3003)

Web control and monitoring interface for BB-8.

**Available pages:**
- `/` — Home page with navigation to modules
- `/video` — Live video feed from the head camera

**Planned modules (in progress):**
- RC Control — real-time keyboard/gamepad teleoperation
- Autonomous mode
- Telemetry dashboard

**Stack:** Next.js 15, TypeScript, Tailwind CSS, Vitest

---

## Stream Client (Next.js — port 3004) + Relay (port 3005)

The streaming client consumes the video feed from the robot's head and makes it available in the webapp through a WebSocket relay server.

### Streaming Architecture

```
Camera (head ESP32)
        │
        ▼
  stream-client (WebSocket producer)
        │  ws://localhost:3005?role=producer
        ▼
     relay.ts (relay server, port 3005)
        │  ws://localhost:3005
        ▼
  webapp video-stream.tsx (consumer)
```

`relay.ts` operates in producer/consumer mode: producers send frames, and the relay broadcasts them to all connected consumers. This decouples the robot's streaming from the number of web clients.

---

## System Requirements

| Dependency | Minimum version | Notes |
|---|---|---|
| Node.js | 18 | Required for webapp and stream client |
| pnpm | latest | Workspace package manager |
| Python | 3.8 | Required for PlatformIO |
| PlatformIO CLI | latest | `pip install platformio` |
| USB access | — | CH340 serial adapter for initial flash |

The development machine and the robot must be on the same local network for teleoperation and OTA updates.

---

## WiFi Configuration

Before the first flash, create the credentials file (gitignored):

```bash
cp code/body-firmware/src/config/wifi_credentials.example.h \
   code/body-firmware/src/config/wifi_credentials.h
```

Then fill in your credentials:

```cpp
#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"
#define OTA_PASSWORD  "your-ota-password"
```

> `wifi_credentials.h` is listed in `.gitignore` and will never be committed.

## Team members

| Name                | Email                                          |
|---------------------|------------------------------------------------|
| Alessandro Lombardini | alessandro.lombardini@epfl.ch                |
| Tristan Compain     | tristan.compain@epfl.ch                        |
| Lucas Brunschwick   | lucas.brunschwick@epfl.ch                      |
| Vincent du Fresne   | vincent.dufresnevonhohenesche@epfl.ch          |
| Alban Thèves        | alban.theves@epfl.ch                           |
