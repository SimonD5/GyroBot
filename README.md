# GyroBot

<p align="center">
  <img src="3D/V1_Images/Global1.png" alt="GyroBot V1 concept" width="480">
</p>

## Overview

**GyroBot** is a compact two-wheeled self-balancing robot designed to operate in environments where mobility, accessibility and autonomous navigation can be challenging.

The project focuses on developing an agile robotic platform combining dynamic balance control with spatial perception (LiDAR and camera), backed by a dedicated remote controller, a wireless repeater node, and a supervisory monitoring station.

## System Architecture

<p align="center">
  <img src="Documentation/Images/Interaction.png" alt="GyroBot System Communication Architecture" width="720">
</p>

The overall system is organized around four cooperating elements:

* **Robot**: Dual-architecture platform powered by an ESP32 for high-frequency balance and motor control (cascade PID at 200 Hz), coupled with a Raspberry Pi for onboard perception (LiDAR, camera) and telemetry transmission.
* **Remote Controller**: A custom handheld controller based on an ESP32, featuring dual joysticks, an OLED display, and dedicated controls for responsive manual steering via ESP-NOW / Wi-Fi.
* **Wi-Fi Repeater**: An autonomous ESP32 wireless relay node deployed to extend transmission range between the robot and the ground station in complex indoor layouts.
* **HMI / Ground Station**: A laptop dashboard displaying real-time environmental maps, sensor telemetry, battery status, and operating metrics.

## Key Targets & Specifications

| Requirement | Current Target | Description / Constraints |
| --- | --- | --- |
| Architecture | 2 coaxial wheels | Inverted pendulum dynamic balance |
| Dimensions & Weight | ~30 cm height, < 1.5 kg | Compact, lightweight chassis |
| Control Loop | 200 Hz (5 ms) | Dual-loop cascade PID (angle stability + velocity drift) |
| Perception | LiDAR + Camera | Environmental mapping (target accuracy ~5 cm) |
| Communication | 2.4 GHz (ESP-NOW / Wi-Fi) | Low-latency steering (< 50 ms) & telemetry |

## Current Progress

* **Functional & Risk Analysis**: Requirements definition, functional breakdown (FAST, Horned Beast, Octopus diagrams), and failure mode analysis (FMEA).
* **Dynamic Simulation**: Webots physics simulation with sensor and motor controllers (`Simulation/`).
* **Hardware Prototyping & Balance Bench**: Physical prototype with ESP32-C6, MPU6050 IMU, motor drivers, and a real-time Wi-Fi cascade PID tuning tool (`Repeteur/Code/`).
* **Remote Controller Design**: Circuit schematic, component selection, and 3D enclosure modeling (`Remote/`).
* **Validation & Test Protocols**: Comprehensive test plan (`TestRobot.md`) covering wireless range, response latency, mapping precision, and balance robustness.

## Project Structure

```text
GyroBot/
├── README.md                          # Project overview and architecture
├── Gantt.xlsx                         # Project planning and milestones
├── 3D/                                # Robot CAD models (STL) and concept renders
├── Documentation/                     # Functional analysis, specifications, and test plans
│   ├── Images/                        # Architecture and functional diagrams
│   ├── HMI/                           # Web interface prototype and mockups
│   └── TestRobot.md                   # System test and validation protocol
├── Remote/                            # Dedicated remote controller
│   ├── 3D/                            # Remote casing designs (V1, V2)
│   ├── ComponentChoiceRemote.xlsx     # Bill of materials and hardware selection
│   └── telecommandeSchematic.png      # Controller wiring schematic
├── Repeteur/                          # Prototype chassis and firmware
│   ├── 3D/                            # Prototype mechanical parts
│   └── Code/                          # Motor/IMU tests & Wi-Fi PID cascade tuning suite
└── Simulation/                        # Webots simulation environment
    ├── controllers/                   # Simulation control code (C++)
    ├── worlds/                        # Webots simulation world
    └── VideoSimulation.mp4            # Demonstration video
```