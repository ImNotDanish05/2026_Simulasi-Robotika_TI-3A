# Architecture

## Overview
Coursework repository for Robotics Simulation (Simulasi Robotika TI-3A) covering both embedded Arduino Uno robotics labs and Webots R2025a E-puck mobile robot simulations.

## System Components

### 1. Arduino Hardware Labs (`Bab-01` .. `Bab-06`)
- **Bab-01_Pemrograman-IO**: Basic GPIO, 4-LED patterns (Active-Low), push button (Active-High), potentiometer input, and hardware/software PWM.
- **Bab-02_Servo**: Servo motor control, 3-button selector (Active-Low with `INPUT_PULLUP`), ultrasonic distance steering via 3-pin SIG mode.
- **Bab-03_DC**: Dual DC motor direction and PWM speed control via L293D motor driver IC.
- **Bab-05_Line-Follower**: Line tracking mobile robot algorithms.
- **Bab-05_Mobil_Avoid_Obstacle_1_SensorUltrasonik**: 1-sensor ultrasonic obstacle-avoiding mobile robot.
- **Bab-06_Mobil_Avoid_Obstacle_3_SensorUltrasonik**: 3-sensor ultrasonic obstacle-avoiding mobile robot.

### 2. Webots Simulation (`Bab-04_WeBots`)
- **Engine**: Webots R2025a mobile robot simulation environment.
- **Robot Model**: E-puck differential drive robot.
  - Actuators: `"left wheel motor"` and `"right wheel motor"` (velocity control with `wb_motor_set_position(m, INFINITY)` + `wb_motor_set_velocity()`).
  - Distance Sensors: 8 IR proximity sensors (`ps0`..`ps7`). Front sensors: `ps0`, `ps7`; corners: `ps1`, `ps6`. Enabled via `wb_distance_sensor_enable(tag, TIME_STEP)`. Effective range: ~7 cm.
  - Light Sensors & LEDs: 8 light sensors (`ls0`..`ls7`), 10 LEDs (`led0`..`led9`).
  - Vision: On-board camera used for color blob/red target detection.
- **Projects / Subdirectories**:
  - `latihan_01/`: Foundational exercises and reactive controllers (`controller_robot_sensor_4.c` right-wall follower).
  - `test_01/`: Trinity/KRPAI 1:2 scale maze project world and controller (`global_robot_controller_sensor.c`).
  - Variasi UTS (`uts_adzi/`, `uts_yafi/`, `uts_abi/`, `uts_ben/`): Customized maze navigation variants with distinct PID/reactive tuning, visual themes, and target detection.
  - `catatan/`: Arena specifications, coordinate mapping (`arena_uts.md`), return home planning (`rencana_solusi_return_home.md`), and sensor prototype notes.
