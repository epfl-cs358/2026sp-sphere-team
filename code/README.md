# Sphere

Monorepo for the Sphere robotics project — ESP32 firmware, web control interface, and streaming client.

## Quick start

```sh
make setup    # install all dependencies
make dev      # start all services
```

## Make targets

| Target | Description |
|--------|-------------|
| `make dev` | Start all services (webapp + stream + relay) |
| `make dev-webapp` | Webapp only (port 3003) |
| `make dev-stream` | Stream client only (port 3004) |
| `make dev-relay` | Relay server only (port 3005) |
| `make dev-stop` | Stop all dev servers (ports 3003-3005) |
| `make test` | Run all tests |
| `make test-firmware` | PlatformIO native tests |
| `make test-firmware-head` | PlatformIO tests on hardware |
| `make test-webapp` | Webapp vitest |
| `make test-stream` | Stream client vitest |
| `make build` | Build everything |
| `make build-webapp` | Build webapp |
| `make build-stream` | Build stream client |
| `make build-firmware` | Compile firmware (no upload) |
| `make lint` | ESLint both web projects |
| `make typecheck` | TypeScript type checking |
| `make flash` | Compile and upload firmware to board |
| `make monitor` | Serial monitor (115200 baud) |
| `make setup` | Full setup (prereqs + web + firmware) |
| `make setup-prereqs` | Check/install system dependencies |
| `make setup-web` | Install pnpm dependencies |
| `make setup-firmware` | Install PlatformIO + firmware libs |
| `make clean` | Remove build artifacts |

## Body firmware: teleop pipeline

Body firmware ships a teleop pipeline:

```
WebSocket client  ──text frame──▶  WebSocketCommandProducer (Core 0)
                                            │
                                            ▼
                                     CommandLatch  (latest-wins, length-1)
                                            │
                                            ▼
                                  controlTask @ 100 Hz (Core 1)
                                            │
                                            ▼
                                  PassthroughDrivetrainController
                                            │
                                            ▼
                                       OmniDrivetrain
```

- Wire format: text frame `"vx,vy,omega"` (m/s, m/s, rad/s). Trailing fields are ignored for forward-compat.
- Build env: `pio run -e robot` (USB flash) or `pio run -e robot_ota -t upload` (OTA).
- Safety: 200 ms staleness ramp-to-zero, 1 s task watchdog on control + WS tasks, WiFi-offline reboot at 30 s.
- Auth: none — plaintext `ws://` on port 80, intended for trusted-LAN operation only.

## OTA updates

The `[env:robot_ota]` PlatformIO env flashes the device over WiFi via ArduinoOTA. Setup:

1. Set `OTA_PASSWORD` in `body-firmware/src/config/wifi_credentials.h` (gitignored).
2. Export the same password in your shell: `export OTA_PASSWORD='your-password'`.
3. `pio run -e robot_ota -t upload` — uploads to `bb8-robot.local` (mDNS) by default.
4. If `.local` name resolution fails, override with `pio run -e robot_ota -t upload --upload-port=<robot-ip>`.

OTA `onStart` halts the producer task before flash is overwritten so motors ramp to zero via the staleness window.
