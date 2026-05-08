# Motor + Encoder Test Procedure

## Prerequisites

- PlatformIO CLI installed (`pip install platformio` or via VS Code extension)
- Wemos D1 UNO32 connected via USB
- 12V power supply connected to L298N boards
- EN jumpered HIGH on all three L298N boards

## Wiring

### Motor Driver (L298N)

| Motor | L298N Channel | fwd (IN3) | rev (IN4) |
|-------|---------------|-----------|-----------|
| 0     | B             | GPIO 14   | GPIO 27   |
| 1     | B             | GPIO 16   | GPIO 17   |
| 2     | B             | GPIO 18   | GPIO 19   |

Each L298N IN3 pin connects to the corresponding ESP32 GPIO in the "fwd" column.
Each L298N IN4 pin connects to the corresponding ESP32 GPIO in the "rev" column.
L298N GND must be connected to ESP32 GND.

### Encoders

| Motor | Encoder A | Encoder B |
|-------|-----------|-----------|
| 0     | GPIO 34   | GPIO 35   |
| 1     | GPIO 4    | GPIO 2    |
| 2     | GPIO 39   | GPIO 36   |

Encoder VCC → 3.3V, GND → GND.

## Flashing

1. `cd code/body-firmware`
2. Plug in the Wemos D1 UNO32 via USB
3. Run:
   ```
   pio run -e wemos_d1_uno32 -t upload
   ```
4. Wait for `SUCCESS` — if it fails with a connection error, hold the BOOT button on the board while it tries to connect, release once upload starts

## Running the Test

1. Open serial monitor:
   ```
   pio device monitor -b 115200
   ```
2. Confirm you see:
   ```
   Motor+encoder test ready. Send: <motor 0-2> <speed -1.0 to 1.0>
   Send 'stop' to brake all, 'rpm' to toggle RPM display.
   ```
3. Type commands in the serial monitor and press Enter

## Commands

| Command | Effect |
|---------|--------|
| `<motor> <speed>` | Set motor (0-2) to speed (-1.0 to 1.0) |
| `stop` | Brake all motors |
| `rpm` | Toggle continuous RPM display (every 20ms) |

## Test 1: Motor 0

| Step | Send | Expected |
|------|------|----------|
| 1 | `0 0.5` | Motor 0 spins forward at half speed |
| 2 | `0 0` | Motor 0 coasts to stop |
| 3 | `0 -0.5` | Motor 0 spins reverse at half speed |
| 4 | `0 0` | Motor 0 coasts to stop |
| 5 | `0 1.0` | Motor 0 spins forward at full speed |
| 6 | `stop` | Motor 0 brakes (stops faster than coast) |

**Result:** PASS / FAIL
**Notes:**

## Test 2: Motor 1

| Step | Send | Expected |
|------|------|----------|
| 1 | `1 0.5` | Motor 1 spins forward at half speed |
| 2 | `1 0` | Motor 1 coasts to stop |
| 3 | `1 -0.5` | Motor 1 spins reverse at half speed |
| 4 | `1 0` | Motor 1 coasts to stop |
| 5 | `1 1.0` | Motor 1 spins forward at full speed |
| 6 | `stop` | Motor 1 brakes (stops faster than coast) |

**Result:** PASS / FAIL
**Notes:**

## Test 3: Motor 2

| Step | Send | Expected |
|------|------|----------|
| 1 | `2 0.5` | Motor 2 spins forward at half speed |
| 2 | `2 0` | Motor 2 coasts to stop |
| 3 | `2 -0.5` | Motor 2 spins reverse at half speed |
| 4 | `2 0` | Motor 2 coasts to stop |
| 5 | `2 1.0` | Motor 2 spins forward at full speed |
| 6 | `stop` | Motor 2 brakes (stops faster than coast) |

**Result:** PASS / FAIL
**Notes:**: inverted direction

## Test 4: Encoder 0

| Step | Send | Expected |
|------|------|----------|
| 1 | `rpm` | RPM display turns ON |
| 2 | `0 0.5` | RPM for [0] shows positive value (~100-125 RPM) |
| 3 | `0 -0.5` | RPM for [0] shows negative value |
| 4 | `0 1.0` | RPM for [0] increases (~200-250 RPM) |
| 5 | `stop` | RPM for [0] drops to ~0 |

**Result:** PASS / FAIL
**Notes:** 0 +-0.5 -> +.10rpm

## Test 5: Encoder 1

| Step | Send | Expected |
|------|------|----------|
| 1 | `1 0.5` | RPM for [1] shows positive value (~100-125 RPM) |
| 2 | `1 -0.5` | RPM for [1] shows negative value |
| 3 | `1 1.0` | RPM for [1] increases (~200-250 RPM) |
| 4 | `stop` | RPM for [1] drops to ~0 |

**Result:** PASS / FAIL
**Notes:**

## Test 6: Encoder 2

| Step | Send | Expected |
|------|------|----------|
| 1 | `2 0.5` | RPM for [2] shows positive value (~100-125 RPM) |
| 2 | `2 -0.5` | RPM for [2] shows negative value |
| 3 | `2 1.0` | RPM for [2] increases (~200-250 RPM) |
| 4 | `stop` | RPM for [2] drops to ~0 |
| 5 | `rpm` | RPM display turns OFF |

**Result:** PASS / FAIL
**Notes:**

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Motor doesn't spin at all | Check wiring to L298N IN3/IN4 and power supply |
| Motor spins but direction is swapped | Swap fwd/rev pin values in `src/config/pins.h` |
| Motor 1 behaves erratically | GPIO 16/17 conflict with PSRAM (only on WROVER modules, not WROOM) |
| Serial shows nothing | Confirm baud rate is 115200, check USB connection |
| Motor spins at full speed regardless of value | EN pin not jumpered HIGH, or IN pins wired to EN |
| Upload fails with connection error | Hold BOOT button on board during upload |
| RPM shows 0 while motor spins | Encoder A/B wires swapped or disconnected |
| RPM sign is opposite to expected | Swap encoder A and B pins in `src/config/pins.h` |
| RPM is very noisy | Check encoder connections, ensure 3.3V power to encoder |
