# Decisions Log

### D-005 — Software PWM via micros() on Arduino Uno pins 12 & 13
- Context: Arduino Uno pins 12 and 13 lack hardware timer PWM channels.
- Decision: Use manual time-slicing via `micros()` loop timing (Bab-01 Program 09) rather than external PWM hardware.
- Revisit if: Migrating to a board with hardware PWM across all pins (e.g. Mega/ESP32).

### D-004 — Red goal box (10x10x10cm) target convention (2026-10-15)
- Context: Lecturer specification for UTS maze target required standardized physical target dimensions.
- Decision: Place red goal box (kotak merah 10x10x10cm) in the maze rooms across UTS worlds (`uts_adzi`, `uts_yafi`, `uts_abi`, `uts_ben`).
- Revisit if: Assignment criteria changes.

### D-003 — Hairpin cornering via extended arc search and dual sensor fusion (2026-10-15)
- Context: E-puck lost wall lock when rounding 180° hairpin wall tips in `controller_yafi.c`.
- Decision: Increase arc search limit to 3.80 rad, reduce inner wheel ratio to 0.08, and fuse side + corner sensor detection.
- Revisit if: Maze geometry or sensor sampling rate changes.

### D-002 — Opsi A (Dual Wall-Following) for maze return home (2026-10-13)
- Context: Need reliable return-home behavior from goal room back to starting position in 1:2 Trinity arena.
- Decision: Implement Opsi A (Dual Wall-Following) in `global_robot_controller_sensor.c` with explicit `RETURN_ENTER_ROOM` state to clear the doorway.
- Alternatives evaluated: Dead reckoning (accumulated wheel drift too high), full floodfill mapping (overkill for simple maze topology). See `catatan/rencana_solusi_return_home.md`.
- Revisit if: Maze contains dynamic or reconfigurable obstacles.

### D-001 — Camera red detection with lower-half ROI and strict color thresholds (2026-10-09)
- Context: Camera falsely triggered on brown wood floor tiles when searching for red goal.
- Decision: Restrict camera ROI to the lower half and use strict RGB thresholding (`r>140, g<65, b<65, r-g>80`) in `controller_robot_sensor_4.c`.
- Revisit if: Lighting conditions or floor textures in the simulation world are modified.
