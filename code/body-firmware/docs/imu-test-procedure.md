# BNO055 IMU Test Procedure

## Prerequisites

- PlatformIO CLI installed
- Wemos D1 UNO32 connected via USB (data cable)
- BNO055 breakout board wired via I2C

## Wiring

| BNO055 Pin | ESP32 Pin |
|------------|-----------|
| VIN        | 3.3V      |
| GND        | GND       |
| SDA        | GPIO 26   |
| SCL        | GPIO 25   |

I2C address: 0x28 (default, ADR pin LOW).

## Flashing

1. `cd code/body-firmware`
2. Plug in the Wemos D1 UNO32 via USB
3. Run:
   ```
   pio run -e wemos_d1_uno32 -t upload
   ```
4. If upload fails with connection error, hold BOOT button during upload

## Running the Test

1. Open serial monitor:
   ```
   pio device monitor -b 115200
   ```
2. Confirm you see:
   ```
   BNO055 IMU test — initializing...
   BNO055 ready. Commands: all, euler, quat, gyro, cal, accel
   ```
3. If you see `ERR: BNO055 not detected`, check I2C wiring

## Commands

| Command | Output |
|---------|--------|
| `euler` | Heading, roll, pitch (degrees) |
| `quat`  | Quaternion (w, x, y, z) |
| `gyro`  | Gyroscope (deg/s) |
| `accel` | Linear acceleration (m/s²) |
| `cal`   | Calibration status (0-3 per subsystem) |
| `all`   | All of the above |

Default display on boot: euler + calibration.

## Test 1: Sensor Detection

| Step | Action | Expected |
|------|--------|----------|
| 1 | Power on, open serial monitor | `BNO055 ready.` message appears |
| 2 | If error, swap SDA/SCL wires and reset | Sensor detected on retry |

**Result:** PASS / FAIL
**Notes:**

## Test 2: Calibration

| Step | Action | Expected |
|------|--------|----------|
| 1 | Send `cal` | Calibration values stream (sys, gyro, accel, mag) |
| 2 | Leave board still on flat surface for 5s | Gyro calibration reaches 3 |
| 3 | Tilt board slowly in multiple directions | Accel calibration reaches 3 |
| 4 | Move board in figure-8 pattern | Mag calibration reaches 3 |
| 5 | Wait until sys=3 | All subsystems calibrated |

**Result:** PASS / FAIL
**Notes:**

## Test 3: Euler Angles

| Step | Action | Expected |
|------|--------|----------|
| 1 | Send `euler` | Euler angles stream (heading, roll, pitch) |
| 2 | Place board flat | Roll and pitch near 0 |
| 3 | Rotate board on table (yaw) | Heading changes 0–360 |
| 4 | Tilt board forward | Pitch changes |
| 5 | Tilt board sideways | Roll changes |
| 6 | Return to flat | Roll and pitch return near 0 |

**Result:** PASS / FAIL
**Notes:**

## Test 4: Quaternion

| Step | Action | Expected |
|------|--------|----------|
| 1 | Send `quat` | Quaternion values stream (w, x, y, z) |
| 2 | Hold board still | Values stable, w near 1.0, others near 0 (if upright) |
| 3 | Rotate board slowly | Values change smoothly, no jumps |

**Result:** PASS / FAIL
**Notes:**

## Test 5: Gyroscope

| Step | Action | Expected |
|------|--------|----------|
| 1 | Send `gyro` | Gyro values stream (x, y, z in deg/s) |
| 2 | Hold board still | All values near 0 |
| 3 | Rotate board around one axis | Corresponding axis shows non-zero reading |
| 4 | Stop rotating | Values return near 0 |

**Result:** PASS / FAIL
**Notes:**

## Test 6: Linear Acceleration

| Step | Action | Expected |
|------|--------|----------|
| 1 | Send `accel` | Acceleration values stream (x, y, z in m/s²) |
| 2 | Hold board still | All values near 0 (gravity is subtracted) |
| 3 | Shake board along one axis | Corresponding axis shows spikes |
| 4 | Hold still again | Values return near 0 |

**Result:** PASS / FAIL
**Notes:**

## Test 7: Calibration Persistence

| Step | Action | Expected |
|------|--------|----------|
| 1 | Calibrate fully (sys=3) | Offsets auto-saved to NVS |
| 2 | Reset board (press reset button) | Board restarts |
| 3 | Send `cal` | Calibration values recover faster than first boot |

**Result:** PASS / FAIL
**Notes:**

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| `ERR: BNO055 not detected` | SDA/SCL swapped, not connected, or wrong I2C address |
| Readings are all zeros | `read()` called before `begin()` succeeded |
| Heading drifts over time | Magnetometer not calibrated (do figure-8 motion) |
| Euler angles jump at 0/360 boundary | Normal gimbal behavior near singularities |
| Calibration doesn't persist after reset | NVS storage issue — check Preferences library |
| Values update slowly | Check that delay in loop is 100ms, not higher |
