# Drivetrain Test Procedure

## Prerequisites

- Motor spin test passing (see `motor-spin-test-procedure.md`)
- All three motors spinning in correct directions with correct RPM readings
- Robot on blocks (wheels off the ground) or in a clear area

## Flashing

```
cd code/body-firmware
pio run -e drivetrain_test -t upload
```

## Serial Monitor

```
pio device monitor -b 115200
```

## Commands

| Command | Effect |
|---------|--------|
| `<vx> <vy> <omega_deg>` | Drive at body velocity (m/s, m/s, deg/s) |
| `stop` | Brake all motors |
| `rpm` | Toggle target vs actual RPM display (T = target, A = actual) |

## Convention

- vx positive = forward
- vy positive = left
- omega positive = CCW from above
- Motor 0 at 0°, Motor 1 at 120°, Motor 2 at 240°

## Test 1: Pure Rotation (CW)

| Step | Send | Expected |
|------|------|----------|
| 1 | `rpm` | RPM display turns ON |
| 2 | `0 0 -90` | Robot rotates CW, all wheels spin at equal RPM |
| 3 | Observe | Target RPMs are all equal and same sign |
| 4 | Observe | Actual RPMs converge to targets within ~1s |
| 5 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 2: Pure Rotation (CCW)

| Step | Send | Expected |
|------|------|----------|
| 1 | `0 0 90` | Robot rotates CCW, all wheels spin (opposite sign to Test 1) |
| 2 | Observe | Actual RPMs converge to targets |
| 3 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 3: Forward (vx only)

| Step | Send | Expected |
|------|------|----------|
| 1 | `0.1 0 0` | Robot moves forward |
| 2 | Observe | Motor 0 target ≈ 0, Motors 1 & 2 equal magnitude opposite sign |
| 3 | Observe | Actual RPMs converge to targets |
| 4 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 4: Backward (vx only)

| Step | Send | Expected |
|------|------|----------|
| 1 | `-0.1 0 0` | Robot moves backward |
| 2 | Observe | Same pattern as Test 3 but signs flipped |
| 3 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 5: Strafe Left (vy only)

| Step | Send | Expected |
|------|------|----------|
| 1 | `0 0.1 0` | Robot moves left |
| 2 | Observe | Motor 0 has largest magnitude, Motors 1 & 2 share remainder |
| 3 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 6: Strafe Right (vy only)

| Step | Send | Expected |
|------|------|----------|
| 1 | `0 -0.1 0` | Robot moves right |
| 2 | Observe | Same pattern as Test 5 but signs flipped |
| 3 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 7: Combined Motion

| Step | Send | Expected |
|------|------|----------|
| 1 | `0.1 0 45` | Robot moves forward while rotating CCW |
| 2 | Observe | Actual RPMs converge to targets |
| 3 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Test 8: PID Tracking Quality

| Step | Send | Expected |
|------|------|----------|
| 1 | `0 0 -90` | Steady rotation |
| 2 | Wait 3s | Actual RPMs settle within ±5% of targets |
| 3 | `0 0 -180` | Step change — RPMs increase |
| 4 | Observe | Actual RPMs converge without excessive overshoot |
| 5 | `stop` | All motors brake |

**Result:** PASS / FAIL
**Notes:**

## Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| Robot drifts instead of pure rotation | One motor wired backwards — recheck pin swap |
| Target RPMs look right but actuals don't converge | PID gains need tuning (Kp=1.0, Ki=0.1, Kd=0.01) |
| Robot moves in wrong direction | Motor numbering doesn't match 0°/120°/240° layout |
| Oscillation / buzzing | Kp too high or Kd too low |
| Slow response | Kp too low or Ki too low |
