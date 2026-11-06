# Current state
Updated: 2026-11-06

## Goal in progress
None currently in progress. Last completed work: implemented E-puck distance capability & wall detection telemetry in UTS `global_robot_controller_sensor.c` and documented `E-puckDistanceSensor.proto` in `catatan/`.

## Done so far
- Arduino Uno labs Bab 01–03 completed.
- Webots E-puck exercises in `Bab-04_WeBots/latihan_01/` and UTS maze simulations in `Bab-04_WeBots/test_01/`.
- Individual UTS variations created (`uts_adzi/`, `uts_yafi/`, `uts_abi/`, `uts_ben/`).
- Telemetry and return home strategies documented in `Bab-04_WeBots/catatan/`.
- New lab directories present in workspace: `Bab-05_Line-Follower/`, `Bab-05_Mobil_Avoid_Obstacle_1_SensorUltrasonik/`, `Bab-06_Mobil_Avoid_Obstacle_3_SensorUltrasonik/`.

## Exactly where I stopped
Finished UTS Webots telemetry implementation and documentation on 2026-10-23. Migrated notes from `DANISH_AI_AGENT.md` to `DANISH_AI_SUMMARY.md` on 2026-11-06.

## Next steps (in order)
1. Awaiting user instructions on which topic or chapter to work on (e.g., exploring/updating Bab 05 Line Follower, Bab 05/06 Obstacle Avoidance, or Webots simulation).

## Half-done / uncommitted changes
- None.

## Findings so far (so files needn't be re-read)
- `Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor/global_robot_controller_sensor.c` — contains real-time distance evaluation and telemetry for the E-puck robot.

## Tried and failed (don't repeat)
- Floor red detection: generic RGB thresholding misidentified brown wood tiles as goal floor. Solution was strict filtering (`r>140, g<65, b<65, r-g>80`) and restricting camera to lower half in `controller_robot_sensor_4.c`.

## Open questions / waiting on the user
- Which task or chapter would you like to proceed with today?

## How to verify
- Webots UTS controller build: `make -C Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor`
