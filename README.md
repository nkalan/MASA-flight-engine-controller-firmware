This project contains the embedded software for the TSM/Clementine Engine Controller PCB, a liquid-fueled rocket in development to launch to 50k feet in 2022. This code was used throughout the summer of 2021 to run active propellant tank pressurization testing and engine coldflow testing on MASA's new RP1-LOX engine. The current code is contained in the `tsm-flight-ec-firmware-pressboard-port` directory.

# Functionality
The TSM Engine Controller is designed to read from the following sensors:
- T-Type thermocouples
- Analog pressure transducers
- Potentiometers

It also controls the following actuators:
- 12V solenoid channels
- Stepper motors

# Architecture
Periodic loops are controlled with timer interrupts.
Telemetry is sent every 100ms, active motor control calculations are performed every 50ms, and sensors are read and logged every 5ms. The state machine controlling the engine's ignition sequence ("autosequence") is constantly checked, and uses the the system state and system time to determine solenoid valve control outputs.
