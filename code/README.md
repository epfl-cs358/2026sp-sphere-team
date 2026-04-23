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
