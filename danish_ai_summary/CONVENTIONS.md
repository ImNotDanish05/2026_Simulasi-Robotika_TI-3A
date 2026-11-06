# Conventions & Gotchas

## Code Conventions
- Bab 1 LEDs are Active-Low (`LOW` = ON, `HIGH` = OFF).
- Button logic varies: Bab 1 uses external Active-High (`INPUT`, `HIGH` = pressed); Bab 2 uses Active-Low (`INPUT_PULLUP`, `LOW` = pressed).
- Ultrasonic sensors use 3-pin SIG mode (`readUltrasonicDistance(pin, pin)`).
- Webots E-puck wheel motors: `"left wheel motor"` and `"right wheel motor"`, MAX_SPEED = 6.28 rad/s.
- E-puck motor modes: Velocity control uses `wb_motor_set_position(m, INFINITY)` + `wb_motor_set_velocity()`; Position control sets specific target radians.
- E-puck sensor devices: 8 infrared distance sensors `"ps0"`..`"ps7"` (front: `ps0`, `ps7`; corners: `ps1`, `ps6`), 8 light sensors `"ls0"`..`"ls7"`, 10 LEDs `"led0"`..`"led9"`. Distance sensors MUST be enabled via `wb_distance_sensor_enable(tag, TIME_STEP)` before calling `wb_distance_sensor_get_value()`. Max IR detection range is ~7.0 cm (lookupTable response 4095 at 0cm down to 67.19 at 7cm).

## Known Gotchas
- Arduino Uno pins 12 & 13 do NOT support hardware PWM; software PWM via `micros()` is required (see Bab-01 Program 09).
- `controller_robot_sensor_4.c` uses a 3-rule reactive right-wall follower integrated with E-puck camera red color detection to stop upon reaching the goal.
- `global_robot_controller_sensor.c` in UTS directory is used for real-time sensor distance evaluation and wall detection telemetry.
- `controller_yafi.c` uses extended arc search limit (3.80 rad), tight inner wheel ratio (0.08), and dual side+corner sensors to safely round 180° hairpin wall tips without losing wall lock.

## DO NOT touch
- `.git/` files
- Compiled controller binaries and `build/` artifact directories

## User Preferences
- Preserve existing comments and docstrings.
- Follow existing patterns in Arduino sketches and Webots controllers without unnecessary refactoring.
