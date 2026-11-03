# Spike scene decisions (WP-S S.4, lead, 2026-11-03)

Recorded with every TST-002 result that uses the spike scene (`.worktrees/spike_repo/`, export-scene of
`ws-s4a-scene-export`).

| # | Decision | Reason |
|---|---|---|
| D1 | Part frame = cell CAD file frame (`model.step` = the cell CAD, part read already placed). `flange_T_part` = Rx 90° + offset from export-scene | DES-001 §5.3 "part = STEP model origin"; D20 / edges.json ids are built on the placed shape |
| D2 | World = file frame rotated +Y → +Z (Rx 90°), origin at the `UR5E/UR7E:1` placement = `base_link` (Assumption until `REF_UR_BASE`, OI-23) | role map, CP-23 |
| D3 | Positioner axis (part axis, belt-driven): world (−2.0, −475.0, 75.0) mm, direction +X (Assumption until `REF_POSITIONER`) | Truong 2026-11-03 |
| D4 | Lower frame `2500001:1` as one convex hull (reaches ≈ 11 mm above the plate top at the corner screws) — kept: conservative, the robot does not reach the plate | review of A3 |
| D5 | No PyYAML in the conda env `awm_occ`; `awm_geometry.miniyaml` (strict subset) reads the role map | CLAUDE.md: do not change the env |
| D6 | Head ↔ UR arm links checked unpadded (MoveIt self-collision), cell / part objects with 3 mm padding | CP-55 |
| D7 | Conduit capsule leaves the head side (x 97, z 110 mm) along tool0 +X — TST-002 says "head rear"; Assumption pending Truong / Nghia | S.4 B |
