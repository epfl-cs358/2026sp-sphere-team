# Risk Assessment — BB-8 Project

## WiFi and/or Bluetooth communication with the ESP32

The robot relies on wireless communication (WiFi and/or Bluetooth) with an external device (controller). The ESP32 supports both protocols, but can always cause unpredictable latency if not treated correctly and range may also become an issue in environments with interference. Since the robot's movement and camera feed both depend on this link, any communication failure directly impacts the core functionality of the project.

## Different voltage requirements across components

The system includes components with very different power needs: DC motors typically run at 6–12V, while the ESP32 operates at 3.3V logic (5V input), and the camera module has its own requirements. Managing multiple voltage rails with converters introduces risks of noise on logic lines, brown-outs during motor startup, and incorrect wiring that could damage components. 

## Magnetic coupling to hold the head

The head of the BB-8 is held onto the body using magnets. The coupling must be strong enough to keep the head attached during acceleration and direction changes, yet free enough to allow smooth rotation in any direction. Finding the right magnet strength, size, and placement is a physical problem that is difficult to reason about purely on paper. Additionally, strong magnets placed near the accelerometer or compass could interfere with sensor readings, introducing errors in the stabilization system.

## Sphere stability and wobble

The internal 3-wheeled chassis pushes the sphere from the inside using friction. If the weight distribution is uneven or the wheels are poorly positioned, the sphere will wobble, spin in place, or veer unpredictably. More critically, sudden acceleration or braking causes the internal chassis to shift momentum abruptly, which can throw the entire system off balance and send the sphere in an unintended direction. Controlling smooth, stable motion therefore requires both a well-balanced mechanical design and a reliable closed-loop control system using the accelerometer — and a failure in either layer will make the robot unusable.

## R5 — Camera performance and feature support

The camera mounted in the head needs to deliver sufficient image quality to support features like face detection. An ESP32-CAM module can handle basic streaming at low frame rates, but real-time face detection using deep learning models requires significantly more processing power. If the camera is connected to the same ESP32 handling motor control, it may saturate the CPU and degrade both systems. The risk is not just whether the camera works, but whether it can run the intended features at an acceptable speed within the constraints of embedded hardware.

## R6 — Keeping the sphere small enough

All internal components(the chassis, motors, batteries, ESP32, wiring, and camera) must fit inside the sphere. The available volume is more constrained than it appears in early sketches. Cables, connectors, and the need for physical access (e.g., to recharge batteries or debug hardware) add hidden bulk. If the sphere is too small, the chassis cannot be built as designed; if it is made larger to compensate, the robot loses its aesthetic and may become harder to control mechanically.

## R7 — Insufficient motor torque

The motors must move the entire robot (chassis, batteries, and all electronics) by pushing against the inside of the sphere. If the motors lack sufficient torque, the robot will struggle to accelerate, climb slight inclines, or recover from perturbations. This risk is compounded by the indirect drive mechanism (friction against the sphere wall), which is inherently less efficient than direct drive. Motor selection must account for the total system weight, the friction coefficient between wheels and sphere, and the expected loads during normal operation.

## R8 — Omni-wheel positioning causing excess friction or mechanical resistance

The three omni-wheels must be positioned at angles that allow movement in any direction while maintaining consistent contact with the inside of the sphere. If the geometry is wrong (wrong angles, wrong contact pressure, or misaligned axles) the wheels will generate lateral friction forces that resist motion rather than enabling it. This is a subtle mechanical design problem: small errors in wheel placement can create binding, uneven wear, or loss of directional control. It is also difficult to diagnose once the chassis is assembled inside the sphere.
