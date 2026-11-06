# Changelog

### 2026-11-06 — Migrated Danish AI Summary structure
- What: Migrated `DANISH_AI_AGENT.md` to modular `DANISH_AI_SUMMARY.md` + `danish_ai_summary/` layout.
- Why: Improved context retention and adherence to Danish AI Summary standards.
- Files: `DANISH_AI_SUMMARY.md`, `danish_ai_summary/CURRENT_STATE.md`, `danish_ai_summary/CONVENTIONS.md`, `danish_ai_summary/ARCHITECTURE.md`, `danish_ai_summary/DECISIONS.md`, `danish_ai_summary/CHANGELOG.md`.
- Verified: All original information preserved and verified against repo structure.

### 2026-10-23 — Implemented E-puck distance capability & wall detection telemetry
- What: Implemented E-puck distance capability & wall detection telemetry in UTS `global_robot_controller_sensor.c` and added `E-puckDistanceSensor.proto` reference in `catatan/`.
- Files: `Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor/global_robot_controller_sensor.c`, `Bab-04_WeBots/catatan/`

### 2026-10-15 — Fixed 180° wall-tip cornering in controller_yafi.c
- What: Fixed 180° wall-tip cornering in `controller_yafi.c` by increasing arc limit to 3.80 rad and combining side+corner sensor detection.
- Files: `Bab-04_WeBots/uts_yafi/controllers/controller_yafi/controller_yafi.c`

### 2026-10-15 — Created distinct UTS variations
- What: Created distinct UTS variations for Yafi (`uts_yafi`), Abi (`uts_abi`), and Ben (`uts_ben`) with unique ASTs, PID tuning, visual themes, and red box targets.
- Files: `Bab-04_WeBots/uts_yafi/`, `Bab-04_WeBots/uts_abi/`, `Bab-04_WeBots/uts_ben/`

### 2026-10-15 — Updated Adzi's UTS world and controller
- What: Updated Adzi's UTS world and controller to use red goal box (kotak merah 10x10x10cm) per lecturer specification.
- Files: `Bab-04_WeBots/uts_adzi/`

### 2026-10-15 — Created distinct UTS variation for Adzi
- What: Created distinct UTS variation for Adzi (`Bab-04_WeBots/uts_adzi/`) with unique textures, modular C architecture, and anti-plagiarism redesign.
- Files: `Bab-04_WeBots/uts_adzi/`

### 2026-10-13 — Added RETURN_ENTER_ROOM state
- What: Added `RETURN_ENTER_ROOM` state in `global_robot_controller_sensor.c` to ensure robot enters fully into start room past the doorway.
- Files: `Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor/global_robot_controller_sensor.c`

### 2026-10-13 — Implemented Opsi A (Dual Wall-Following)
- What: Implemented Opsi A (Dual Wall-Following) in `global_robot_controller_sensor.c` and documented 3 return strategies in `catatan/rencana_solusi_return_home.md`.
- Files: `Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor/global_robot_controller_sensor.c`, `Bab-04_WeBots/catatan/rencana_solusi_return_home.md`

### 2026-10-13 — Built 1:2 scale UTS Trinity/KRPAI maze
- What: Built 1:2 scale UTS Trinity/KRPAI maze in `Bab-04_WeBots/test_01/worlds/test_01.wbt` with E-puck and target.
- Files: `Bab-04_WeBots/test_01/worlds/test_01.wbt`

### 2026-10-13 — Documented UTS arena specs & Webots coordinate mapping
- What: Documented UTS arena specs & Webots coordinate mapping in `Bab-04_WeBots/catatan/arena_uts.md` from `arena uts.docx`.
- Files: `Bab-04_WeBots/catatan/arena_uts.md`

### 2026-10-09 — Made floor red detection strict
- What: Made floor red detection strict (`r>140, g<65, b<65, r-g>80`) to reject brown wood tiles in `controller_robot_sensor_4.c`.
- Files: `Bab-04_WeBots/latihan_01/controllers/controller_robot_sensor_4/controller_robot_sensor_4.c`

### 2026-10-09 — Focused camera red detection to lower half
- What: Focused camera red detection to lower half in `controller_robot_sensor_4.c`.
- Files: `Bab-04_WeBots/latihan_01/controllers/controller_robot_sensor_4/controller_robot_sensor_4.c`

### 2026-10-09 — Verified working elegant 3-rule right-wall follower
- What: Verified working elegant 3-rule right-wall follower in `controller_robot_sensor_4.c`.
- Files: `Bab-04_WeBots/latihan_01/controllers/controller_robot_sensor_4/controller_robot_sensor_4.c`

### 2026-10-09 — Added obstacle stop logic
- What: Added obstacle stop logic (`OBSTACLE_THRESHOLD 80.0`) in `controller_robot_sensor.c`.
- Files: `Bab-04_WeBots/latihan_01/controllers/controller_robot_sensor/controller_robot_sensor.c`

### 2026-10-09 — Initialized DANISH_AI_AGENT.md context
- What: Initialized `DANISH_AI_AGENT.md` context after repository-wide scan.
