# Risk Assessment

## WiFi and/or Bluetooth communication with the ESP32

The robot relies on wireless communication (WiFi and/or Bluetooth) with an external device (controller). The ESP32 supports both protocols, but can always cause unpredictable latency if not treated correctly and range may also become an issue in environments with interference. Since the robot's movement and camera feed both depend on this link, any communication failure directly impacts the core functionality of the project.

### Solutions
We will start by building a dedicated communication prototype in the first weeks we recieve the compenents: two ESP32s exchanging movement commands in a loop, where we measure latency and packet loss under realistic conditions. We will pick one primary protocol (most likely WiFi UDP for its low latency) and avoid running both WiFi and Bluetooth simultaneously unless strictly necessary.

## Different voltage requirements across components

The system includes components with very different power needs: DC motors typically run at 6–12V, while the ESP32 operates at 3.3V logic (5V input), and the camera module has its own requirements. Managing multiple voltage rails with converters introduces risks of noise on logic lines, brown-outs during motor startup, and incorrect wiring that could damage components. 

### Solutions
Before wiring anything, we will draw a complete power distribution schematic listing every component, its required voltage, and its peak current draw. We will then build a power prototype early on (wiring all consumers (ESP32, motors, camera) on the real batteries with the planned voltage converters) and verify that the ESP32 does not reset when the motors start. We will add decoupling capacitors on every power line.

## Magnetic coupling to hold the head

The head of the BB-8 is held onto the body using magnets. The coupling must be strong enough to keep the head attached during acceleration and direction changes, yet free enough to allow smooth rotation in any direction. Finding the right magnet strength, size, and placement is a physical problem that is difficult to reason about purely on paper. Additionally, strong magnets placed near the accelerometer or compass could interfere with sensor readings, introducing errors in the stabilization system.

### Solutions
We made several calculations on the head weight and the required magnets strenght that should guarentee good results.

## Sphere stability and wobble

The internal 3-wheeled chassis pushes the sphere from the inside using friction. If the weight distribution is uneven or the wheels are poorly positioned, the sphere will wobble, spin in place, or veer unpredictably. More critically, sudden acceleration or braking causes the internal chassis to shift momentum abruptly, which can throw the entire system off balance and send the sphere in an unintended direction. Controlling smooth, stable motion therefore requires both a well-balanced mechanical design and a reliable closed-loop control system using the accelerometer — and a failure in either layer will make the robot unusable.

### Solutions
To address sphere stability, we will place the heaviest components as low as possible on the chassis to keep the center of mass close to the ground, which passively reduces wobble. We will then use an accelerometer combined with a software control loop to continuously monitor and adjust the motor commands, compensating for unwanted movements in real time.

## Camera performance and feature support

The camera mounted in the head needs to deliver sufficient image quality to support features like face detection. An ESP32-CAM module can handle basic streaming at low frame rates, but real-time face detection using deep learning models requires significantly more processing power. If the camera is connected to the same ESP32 handling motor control, it may saturate the CPU and degrade both systems. The risk is not just whether the camera works, but whether it can run the intended features at an acceptable speed within the constraints of embedded hardware.

### Solutions
We will prototype the camera system completely independently, getting face detection running on a standalone device before any robot integration. We will use a dedicated processor for vision (a separate ESP32) so it never competes with motor control for CPU time. If on-board inference is too slow for real-time detection, we will offload the computation to an external laptop over WiFi: the robot streams video out, the laptop runs the detection model and sends back high-level commands. 

## Keeping the sphere small enough

All internal components(the chassis, motors, batteries, ESP32, wiring, and camera) must fit inside the sphere. The available volume is more constrained than it appears in early sketches. Cables, connectors, and the need for physical access (e.g., to recharge batteries or debug hardware) add hidden bulk. If the sphere is too small, the chassis cannot be built as designed; if it is made larger to compensate, the robot loses its aesthetic and may become harder to control mechanically.

### Solutions
This has already been done on the cad. We have left some margin for mistake.

## Insufficient motor torque

The motors must move the entire robot (chassis, batteries, and all electronics) by pushing against the inside of the sphere. If the motors lack sufficient torque, the robot will struggle to accelerate, climb slight inclines, or recover from perturbations. This risk is compounded by the indirect drive mechanism (friction against the sphere wall), which is inherently less efficient than direct drive. Motor selection must account for the total system weight, the friction coefficient between wheels and sphere, and the expected loads during normal operation.

### Solutions

We already estimated the required torque before ordering motors: calculating the total system weight and the expected friction force between wheels and sphere, and selecting motors that exceed this with clear margin. We will then build the chassis at its intended full weight and verify on a flat surface that the motors can accelerate and stop without stalling. If this isn't enough, we will add/create some kind of gear to adjust the power.

## Omni-wheel positioning causing excess friction or mechanical resistance

The three omni-wheels must be positioned at angles that allow movement in any direction while maintaining consistent contact with the inside of the sphere. If the geometry is wrong (wrong angles, wrong contact pressure, or misaligned axles) the wheels will generate lateral friction forces that resist motion rather than enabling it. This is a subtle mechanical design problem: small errors in wheel placement can create binding, uneven wear, or loss of directional control. It is also difficult to diagnose once the chassis is assembled inside the sphere.

### Solutions
We calulated the angles with the CAD and will print our own omni-wheels so we will be able to adjust them tout our specific requirements.

## Sphere structural integrity and cracking

The sphere is a critical structural component that must withstand continuous mechanical stress from internal forces, impacts with the environment, and repeated opening/closing if the system is accessible. Depending on the material (e.g., PLA, PETG, polycarbonate), the sphere may be prone to cracking, especially along layer lines if 3D printed. Small defects or stress concentrations (such as holes for ventilation or assembly seams) can propagate into larger fractures over time. A cracked sphere not only compromises the aesthetic but can completely break the locomotion system, as the internal traction relies on a smooth and rigid surface.

### Solutions
We made sevral tests with smaller 3-D printed spheres and talked with Sebastien who told us the integrety shouldn't be a problem.

## Imperfect sphere geometry

If the sphere is not perfectly round (due to manufacturing tolerances, deformation, poor assembly, or uneven material thickness), the robot may experience irregular motion. Even small deviations in curvature can change how the omni-wheels contact the surface, creating vibration, wobble, or inconsistent rolling behavior. Since the entire locomotion concept depends on predictable contact between the wheels and the sphere, geometric imperfections can significantly degrade performance.

### Solutions
We will validate the sphere's roundness before assembling the chassis inside it: rolling it on a flat surface and checking for wobble, and measuring the internal diameter at multiple points. Once 3-D printed, we will sand and finish the internal surface to reduce irregularities.
