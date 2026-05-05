# Motor Spin Test Procedure

## Prerequisites

- PlatformIO CLI installed (`pip install platformio` or via VS Code extension)
- Wemos D1 UNO32 connected via USB
- 12V power supply connected to L298N boards
- EN jumpered HIGH on all three L298N boards

## Wiring

| Motor | L298N Channel | fwd (IN3) | rev (IN4) |
|-------|---------------|-----------|-----------|
| 0     | B             | GPIO 27   | GPIO 14   |
| 1     | B             | GPIO 16   | GPIO 17   |
| 2     | B             | GPIO 18   | GPIO 19   |

Each L298N IN3 pin connects to the corresponding ESP32 GPIO in the "fwd" column.
Each L298N IN4 pin connects to the corresponding ESP32 GPIO in the "rev" column.
L298N GND must be connected to ESP32 GND.

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
   Motor test ready. Send: <motor 0-2> <speed -1.0 to 1.0>
   Send 'stop' to brake all.
   ```
3. Type commands in the serial monitor and press Enter

## Commands

| Command | Effect |
|---------|--------|
| `<motor> <speed>` | Set motor (0-2) to speed (-1.0 to 1.0) |
| `stop` | Brake all motors |

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
