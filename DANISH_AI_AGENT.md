# DANISH_AI_AGENT.md
> This file is for AI agents, not humans. Update after finishing every task. Read this BEFORE exploring other files.

## Overview
Coursework repository for Robotics Simulation (Simulasi Robotika TI-3A), containing Arduino Uno hardware lab sketches and Webots R2025a E-puck mobile robot simulations.

## Stack
- Arduino Uno (`.ino` sketches for Tinkercad / physical board)
- C (Webots R2025a robot controllers with GNU Make)

## Commands
- Build Webots controller: `make -C Bab-04_WeBots/latihan_01/controllers/<controller_dir>`
- Build Webots UTS controller: `make -C Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor`
- Clean Webots build: `make -C <controller_dir> clean`
- Run Webots simulation: `webots Bab-04_WeBots/latihan_01/worlds/latihan_01.wbt`
- Run Webots UTS simulation: `webots Bab-04_WeBots/test_01/worlds/test_01.wbt`

## Structure map (top-level only, 1 line per folder)
- `Bab-01_Pemrograman-IO/` — 4-LED patterns, push button, potentiometer, and hybrid PWM
- `Bab-02_Servo/` — Servo motor 3-button selector and ultrasonic distance steering
- `Bab-03_DC/` — Dual DC motor direction and PWM speed control via L293D driver
- `Bab-04_WeBots/` — Webots simulation: `latihan_01/` (dasar), `test_01/` (Project UTS Trinity/KRPAI 1:2), variasi UTS (`uts_adzi/`, `uts_yafi/`, `uts_abi/`, `uts_ben/`), `catatan/` (spesifikasi arena & solusi)

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
- `controller_robot_sensor_4.c` uses a 3-rule reactive right-wall follower integrated with E-puck camera red color detection to stop upon reaching the goal.
- `global_robot_controller_sensor.c` implements Opsi A (Dual Wall-Following Left-Wall return) with `RETURN_ENTER_ROOM` to drive fully inside the start room.
- `controller_yafi.c` uses extended arc search limit (3.80 rad), tight inner wheel ratio (0.08), and dual side+corner sensors to safely round 180° hairpin wall tips without losing wall lock.

## Changelog (newest first, 1 line per entry, NOT a diff)
- 2026-10-15: Fixed 180° wall-tip cornering in controller_yafi.c by increasing arc limit to 3.80 rad and combining side+corner sensor detection.
- 2026-10-15: Created distinct UTS variations for Yafi (uts_yafi), Abi (uts_abi), and Ben (uts_ben) with unique ASTs, PID tuning, visual themes, and red box targets.
- 2026-10-15: Updated Adzi's UTS world and controller to use red goal box (kotak merah 10x10x10cm) per lecturer specification.
- 2026-10-15: Created distinct UTS variation for Adzi (Bab-04_WeBots/uts_adzi/) with unique textures, modular C architecture, and anti-plagiarism redesign.
- 2026-10-13: Added RETURN_ENTER_ROOM state in global_robot_controller_sensor.c to ensure robot enters fully into start room past the doorway.
- 2026-10-13: Implemented Opsi A (Dual Wall-Following) in global_robot_controller_sensor.c and documented 3 return strategies in catatan/rencana_solusi_return_home.md.
- 2026-10-13: Built 1:2 scale UTS Trinity/KRPAI maze in Bab-04_WeBots/test_01/worlds/test_01.wbt with E-puck and target.
- 2026-10-13: Documented UTS arena specs & Webots coordinate mapping in Bab-04_WeBots/catatan/arena_uts.md from arena uts.docx.
- 2026-10-09: Made floor red detection strict (r>140, g<65, b<65, r-g>80) to reject brown wood tiles in controller_robot_sensor_4.c.
- 2026-10-09: Focused camera red detection to lower half in controller_robot_sensor_4.c.
- 2026-10-09: Verified working elegant 3-rule right-wall follower in controller_robot_sensor_4.c.
- 2026-10-09: Added obstacle stop logic (OBSTACLE_THRESHOLD 80.0) in controller_robot_sensor.c.
- 2026-10-09: Initialized DANISH_AI_AGENT.md context after repository-wide scan.

