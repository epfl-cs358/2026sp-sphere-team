# FIT0186Motor Concrete Implementation — Design Spec

## Context

The body-firmware project has a complete set of abstract interfaces but no concrete implementations. The team has received the physical hardware (BTS7960 H-bridge drivers + FIT0186 DC geared motors with Hall-effect quadrature encoders). This spec covers:

1. Reorganizing `src/` into domain subdirectories (committed on main)
2. Implementing the concrete motor stack: `Driver` abstraction, `BTS7960Driver`, and `FIT0186Motor` (on a feature branch)

## Phase 1: Project Setup (on main)

Two commits: directory reorganization, then testing infrastructure.

### Target layout

```
src/
  main.cpp
  config/
  motor/
    Motor.h
    constants/
    driver/
  drivetrain/
    Drivetrain.h
    controller/
      DrivetrainController.h
  imu/
    IMU.h
  input/
    InputSource.h
    CommandBuffer.h
  control/
    PID.h
  util/
```

### Commit 1: Directory reorganization

- `git mv` each header to its new location
- Update all `#include` paths in `main.cpp`
- Add `-I` flags to `platformio.ini` for each subdirectory:
  ```
  -Isrc/config -Isrc/motor -Isrc/motor/constants -Isrc/motor/driver -Isrc/drivetrain -Isrc/drivetrain/controller -Isrc/imu -Isrc/input -Isrc/control -Isrc/util
  ```
- Verify `pio run` compiles

### Commit 2: Testing infrastructure

Add a `[env:native]` environment for running tests on the host machine (no ESP32):

```ini
[env:native]
platform = native
build_flags =
    -std=gnu++17
    -Wall
    -Wextra
    -Isrc/config
    -Isrc/motor
    -Isrc/motor/constants
    -Isrc/motor/driver
    -Isrc/drivetrain
    -Isrc/drivetrain/controller
    -Isrc/imu
    -Isrc/input
    -Isrc/control
    -Isrc/util
    -DBB8_DEBUG
```

PlatformIO uses Unity (C test framework) by default for native tests. Tests live in `test/`:

```
test/
  test_native/        — tests that run on host (pure logic, mocks)
```

Verify with `pio test -e native`.

## Phase 2: Motor Stack Implementation (on feature branch)

Branch: `feature/motor-impl`

### Updated Motor Interface

`src/motor/Motor.h`:

```cpp
class Motor {
public:
    virtual ~Motor() = default;
    virtual void begin() = 0;
    virtual void update() = 0;               // refresh encoder/RPM state; call at fixed interval
    virtual void setSpeed(float speed) = 0;  // [-1.0, 1.0], 0.0 = coast
    virtual void brake() = 0;                // active stop (shorts terminals)
    virtual float getRPM() = 0;              // raw, cached from last update()
    virtual float getFilteredRPM() = 0;      // smoothed via moving average
};
```

- `begin()` — deferred hardware init (Arduino pattern; ESP32 peripherals not ready at static construction time). Seeds timing state (`_lastUpdateMicros`, `_lastCount`) so the first `update()` produces RPM = 0, not garbage.
- `update()` — refresh encoder readings and compute RPM. Called at a fixed interval from the control loop (via `Drivetrain::update()`).
- `setSpeed(0)` — coast (both H-bridge outputs LOW, motor disconnected)
- `brake()` — active braking (both H-bridge outputs HIGH, terminals shorted)
- `getRPM()` / `getFilteredRPM()` — return cached values computed by last `update()` call

**Convention:** All member variables use leading underscore (`_member`) to match existing codebase.

### New Files

#### `src/config/debug.h`

Project-wide defensive programming macros:

- `BB8_ASSERT(cond, msg)` — in debug: logs via ArduinoLog (`Log.fatal`) with file/line, then calls `abort()` (triggers ESP32 panic handler with backtrace). In release (`#ifndef BB8_DEBUG`): compiles to `((void)0)`.
- Uses ArduinoLog for all output. ArduinoLog itself is stripped in release via `DISABLE_LOGGING`.

#### `src/config/pins.h`

```cpp
#include "MotorConfig.h"
#include "FIT0186.h"

// Motor 0
inline constexpr MotorConfig MOTOR0 {
    .driverPins = { ... },
    .encoderPins = { ... },
    .encoderCPR = fit0186::ENCODER_CPR
};
// Motor 1
inline constexpr MotorConfig MOTOR1 {
    .driverPins = { ... },
    .encoderPins = { ... },
    .encoderCPR = fit0186::ENCODER_CPR
};
// Motor 2
inline constexpr MotorConfig MOTOR2 {
    .driverPins = { ... },
    .encoderPins = { ... },
    .encoderCPR = fit0186::ENCODER_CPR
};
```

Pin values TBD based on physical wiring. Motor constants come from `motor/constants/FIT0186.h`. Structure allows changing all config in one file.

#### `src/motor/MotorConfig.h`

```cpp
struct DriverPins {
    uint8_t rpwm;      // H-bridge forward PWM pin
    uint8_t lpwm;      // H-bridge reverse PWM pin
};

struct EncoderPins {
    uint8_t a;          // Quadrature channel A
    uint8_t b;          // Quadrature channel B
};

struct MotorConfig {
    DriverPins driverPins;
    EncoderPins encoderPins;
    uint16_t encoderCPR;  // counts per revolution at gearbox output
};
```

#### `src/motor/constants/FIT0186.h`

Hardware constants for the DFRobot FIT0186 motor:

```cpp
#pragma once
#include <cstdint>

namespace fit0186 {
    inline constexpr uint16_t ENCODER_CPR = 700;  // counts per revolution at gearbox output
    inline constexpr float GEAR_RATIO = 43.8f;
    inline constexpr uint16_t NO_LOAD_RPM = 251;
}
```

#### `src/motor/driver/Driver.h`

Abstract H-bridge driver interface:

```cpp
class Driver {
public:
    virtual ~Driver() = default;
    virtual void begin() = 0;
    virtual void setOutput(float value) = 0;  // [-1.0, 1.0], 0.0 = coast
    virtual void brake() = 0;                 // active stop
};
```

#### `src/motor/driver/BTS7960Driver.h` / `BTS7960Driver.cpp`

Concrete `Driver` implementation wrapping the Arduino-BTS7960 library.

**Constructor** — two overloads:
- `BTS7960Driver(DriverPins pins)` — delegates to individual-arg constructor
- `BTS7960Driver(uint8_t rpwm, uint8_t lpwm)`
- Stores pin config, does NOT touch hardware

**begin()** — initializes BTS7960 PWM channels

**setOutput(float value)** — `BB8_ASSERT` that value is in [-1.0, 1.0]. Maps normalized float to BTS7960 PWM. Positive = RPWM, negative = LPWM, zero = both LOW (coast).

**brake()** — drives both RPWM and LPWM HIGH to short motor terminals

#### `src/motor/FIT0186Motor.h` / `FIT0186Motor.cpp`

Concrete `Motor` implementation. Owns a `Driver&` reference and an `ESP32Encoder` instance.

**Class template**: `template <size_t FilterN = 5> class FIT0186Motor : public Motor`

FilterN is the moving average window size (compile-time). Default of 5 samples.

**Constructor** — two overloads:
- `FIT0186Motor(Driver& driver, EncoderPins encoderPins, uint16_t encoderCPR)`
- `FIT0186Motor(Driver& driver, MotorConfig config)` — extracts encoder fields from config
- Stores config, does NOT touch hardware

**begin()** — calls `_driver.begin()`, attaches ESP32Encoder on encoder pins. Seeds `_lastUpdateMicros = micros()` and `_lastCount = encoder.getCount()` so first `update()` produces RPM = 0.

**update()** — tracks timing via `micros()` delta. Snapshots encoder count (`int64_t`), computes RPM:
```
now = micros()
dtSeconds = (now - _lastUpdateMicros) / 1'000'000.0f
deltaCount = encoder.getCount() - _lastCount  // int64_t arithmetic
rpm = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR)) * (60.0f / dtSeconds)
```
Pushes raw RPM into MovingAverage. `_lastCount` is `int64_t` to match `ESP32Encoder::getCount()`.

**setSpeed(float speed)** — delegates to `_driver.setOutput(speed)`

**brake()** — delegates to `_driver.brake()`

**getRPM()** — returns last raw RPM (cached from update())

**getFilteredRPM()** — returns MovingAverage output (returns 0.0f if no samples yet)

**Lifetime requirement:** `Driver&` passed to constructor must outlive the `FIT0186Motor` instance.

#### `src/util/MovingAverage.h`

Header-only template class:

```cpp
template <size_t N>
class MovingAverage {
    static_assert(N > 0, "Window size must be at least 1");
public:
    void push(float value);
    float average() const;  // returns 0.0f when empty
    void reset();
private:
    std::array<float, N> _buffer{};
    size_t _index = 0;
    size_t _count = 0;
};
```

Compile-time window size via template parameter. Reusable for IMU smoothing, PID derivative filtering, etc.

### platformio.ini Changes

```ini
build_flags =
    -std=gnu++17
    -Wall
    -Wextra
    -Isrc/config
    -Isrc/motor
    -Isrc/motor/constants
    -Isrc/motor/driver
    -Isrc/drivetrain
    -Isrc/drivetrain/controller
    -Isrc/imu
    -Isrc/input
    -Isrc/control
    -Isrc/util
    -DBB8_DEBUG
build_unflags = -std=gnu++11
lib_deps =
    luisllamasbinaburo/Arduino-BTS7960
    madhephaestus/ESP32Encoder
    thijse/ArduinoLog
```

Remove `-DBB8_DEBUG` for release builds. ArduinoLog stripped via `DISABLE_LOGGING` define when needed.

### Hardware Reference

- **Motor**: DFRobot FIT0186 — 12V, 251 RPM no-load, 7A stall, 43.8:1 gear ratio
- **Encoder**: Hall-effect quadrature, 16 CPR motor shaft / 700 CPR gearbox shaft
- **Driver**: BTS7960 H-bridge, PWM + direction control
- **RPM resolution**: at 251 RPM and 10ms update interval, ~29 encoder ticks per sample

## Verification

### Phase 1 (project setup)

**Commit 1 — reorganization:**
- `pio run` compiles with no errors/warnings
- All `#include` directives in `main.cpp` resolve to the new paths
- No headers left in `src/` root (except `main.cpp`)
- Each subdirectory contains the expected files per the target layout

**Commit 2 — testing infrastructure:**
- `[env:native]` added to `platformio.ini` with matching `-I` flags and `-std=gnu++17`
- `test/test_native/` directory exists with a minimal test file (e.g. a single passing assertion)
- `pio test -e native` runs the test and reports success
- `pio run` (ESP32 env) still compiles without regression

### Phase 2 (Motor implementation — TDD)
Native tests (`pio test -e native`):
- MovingAverage: push/average, empty returns 0.0f, reset, wraparound
- RPM calculation logic with known encoder deltas
- FIT0186Motor with MockDriver: setSpeed delegates, brake delegates, update computes correct RPM
- BB8_ASSERT fires on out-of-range setOutput in debug build

Hardware verification (upload to ESP32):
- `begin()` initializes without crash
- `setSpeed(0.5)` spins a motor forward
- `setSpeed(-0.5)` spins reverse
- `setSpeed(0)` coasts
- `brake()` stops quickly
- `getRPM()` returns non-zero while spinning
- `getFilteredRPM()` returns a smoother curve than `getRPM()`
