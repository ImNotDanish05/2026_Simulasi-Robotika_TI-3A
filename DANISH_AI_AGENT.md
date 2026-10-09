# DANISH_AI_AGENT.md
> This file is for AI agents, not humans. Update after finishing every task. Read this BEFORE exploring other files.

## Overview
Coursework repository for Robotics Simulation (Simulasi Robotika TI-3A), containing Arduino Uno hardware lab sketches and Webots R2025a E-puck mobile robot simulations.

## Stack
- Arduino Uno (`.ino` sketches for Tinkercad / physical board)
- C (Webots R2025a robot controllers with GNU Make)

## Commands
- Build Webots controller: `make -C Bab-04_WeBots/latihan_01/controllers/<controller_dir>`
- Clean Webots build: `make -C Bab-04_WeBots/latihan_01/controllers/<controller_dir> clean`
- Run Webots simulation: `webots Bab-04_WeBots/latihan_01/worlds/latihan_01.wbt`

## Structure map (top-level only, 1 line per folder)
- `Bab-01_Pemrograman-IO/` — 4-LED patterns, push button, potentiometer, and hybrid PWM
- `Bab-02_Servo/` — Servo motor 3-button selector and ultrasonic distance steering
- `Bab-03_DC/` — Dual DC motor direction and PWM speed control via L293D driver
- `Bab-04_WeBots/` — Webots R2025a simulation worlds, E-puck PROTO reference, and C controllers

## Conventions that differ from defaults / can't be inferred
- Bab 1 LEDs are Active-Low (`LOW` = ON, `HIGH` = OFF).
- Button logic varies: Bab 1 uses external Active-High (`INPUT`, `HIGH` = pressed); Bab 2 uses Active-Low (`INPUT_PULLUP`, `LOW` = pressed).
- Ultrasonic sensors use 3-pin SIG mode (`readUltrasonicDistance(pin, pin)`).
- Webots E-puck wheel motors: `"left wheel motor"` and `"right wheel motor"`, MAX_SPEED = 6.28 rad/s.
- E-puck motor modes: Velocity control uses `wb_motor_set_position(m, INFINITY)` + `wb_motor_set_velocity()`; Position control sets specific target radians.
- E-puck sensor devices: 8 infrared distance sensors `"ps0"`..`"ps7"` (front: `ps0`, `ps7`; corners: `ps1`, `ps6`), 8 light sensors `"ls0"`..`"ls7"`, 10 LEDs `"led0"`..`"led9"`. Distance sensors MUST be enabled via `wb_distance_sensor_enable(tag, TIME_STEP)` before calling `wb_distance_sensor_get_value()`.

## DO NOT touch
- `.git/` files
- Compiled controller binaries and `build/` artifact directories

## Known gotchas
- Arduino Uno pins 12 & 13 do NOT support hardware PWM; software PWM via `micros()` is required (see Bab-01 Program 09).
- `controller_robot_sensor_4.c` uses continuous reactive right-wall following with arc cornering (`MAX_SPEED, 0.25 * MAX_SPEED`).

## Changelog (newest first, 1 line per entry, NOT a diff)
- 2026-10-09: Reverted controller_robot_sensor_4.c back to smooth reactive wall follower with arc cornering.
- 2026-10-09: Implemented smooth reactive right-wall follower in controller_robot_sensor_4.c.
- 2026-10-09: Fixed syntax error (dangling ||) and variable names (psX_val) in controller_robot_sensor_4.c.
- 2026-10-09: Updated controller_robot_sensor_3 with right-side sensor ps2 and left-turn avoidance logic.
- 2026-10-09: Added obstacle stop logic (OBSTACLE_THRESHOLD 80.0) in controller_robot_sensor.c.
- 2026-10-09: Initialized DANISH_AI_AGENT.md context after repository-wide scan.

