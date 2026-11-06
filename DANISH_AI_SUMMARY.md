# DANISH_AI_SUMMARY.md
> Written by AI agents, for AI agents. Read this, then danish_ai_summary/CURRENT_STATE.md, BEFORE exploring other files.
> Detailed notes live in danish_ai_summary/. Update them as you work, not just at the end.

## Overview
Coursework repository for Robotics Simulation (Simulasi Robotika TI-3A), containing Arduino Uno hardware lab sketches and Webots R2025a E-puck mobile robot simulations.

## Must-know
- Bab 1 LEDs are Active-Low (`LOW` = ON, `HIGH` = OFF).
- Button logic varies: Bab 1 uses external Active-High (`INPUT`, `HIGH` = pressed); Bab 2 uses Active-Low (`INPUT_PULLUP`, `LOW` = pressed).
- Arduino Uno pins 12 & 13 do NOT support hardware PWM; software PWM via `micros()` is required (see Bab-01 Program 09).
- Ultrasonic sensors use 3-pin SIG mode (`readUltrasonicDistance(pin, pin)`).
- Webots E-puck wheel motors: `"left wheel motor"` and `"right wheel motor"`, MAX_SPEED = 6.28 rad/s.
- Webots velocity control requires `wb_motor_set_position(m, INFINITY)` + `wb_motor_set_velocity()`.
- Distance sensors (`"ps0"`..`"ps7"`) MUST be enabled via `wb_distance_sensor_enable(tag, TIME_STEP)` before calling `wb_distance_sensor_get_value()`. Max IR detection range is ~7.0 cm.

## Stack
- Arduino Uno (`.ino` sketches for Tinkercad / physical board)
- C (Webots R2025a robot controllers with GNU Make)

## Commands
- Build Webots controller: `make -C Bab-04_WeBots/latihan_01/controllers/<controller_dir>`
- Build Webots UTS controller: `make -C Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor`
- Clean Webots build: `make -C <controller_dir> clean`
- Run Webots simulation: `webots Bab-04_WeBots/latihan_01/worlds/latihan_01.wbt`
- Run Webots UTS simulation: `webots Bab-04_WeBots/test_01/worlds/test_01.wbt`

## Structure map (top-level, one line per folder)
- `Bab-01_Pemrograman-IO/` — 4-LED patterns, push button, potentiometer, and hybrid PWM
- `Bab-02_Servo/` — Servo motor 3-button selector and ultrasonic distance steering
- `Bab-03_DC/` — Dual DC motor direction and PWM speed control via L293D driver
- `Bab-04_WeBots/` — Webots simulation: `latihan_01/` (dasar), `test_01/` (Project UTS Trinity/KRPAI 1:2), variasi UTS (`uts_adzi/`, `uts_yafi/`, `uts_abi/`, `uts_ben/`), `catatan/` (spesifikasi arena & solusi)
- `Bab-05_Line-Follower/` — Line follower robot sketches
- `Bab-05_Mobil_Avoid_Obstacle_1_SensorUltrasonik/` — Obstacle avoidance robot with 1 ultrasonic sensor
- `Bab-06_Mobil_Avoid_Obstacle_3_SensorUltrasonik/` — Obstacle avoidance robot with 3 ultrasonic sensors

## Notes index (what to read when)
| Need | Read |
|---|---|
| Where did we stop? | danish_ai_summary/CURRENT_STATE.md |
| Big picture | danish_ai_summary/ARCHITECTURE.md |
| Before writing code | danish_ai_summary/CONVENTIONS.md |
| Why is it like this? | danish_ai_summary/DECISIONS.md |
| Recent changes | danish_ai_summary/CHANGELOG.md |
| Area deep dives | danish_ai_summary/modules/<area>.md |
| Older history | danish_ai_summary/archive/ (only if needed) |

## DO NOT touch
- `.git/` files
- Compiled controller binaries and `build/` artifact directories

<!-- last verified: 2026-11-06 -->
