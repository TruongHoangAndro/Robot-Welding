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
| D8 | Collision representation per component: convex hull by default, tessellated mesh where the map says `mesh`; `Jig:1` = mesh (its hull covered J1 by ≈ 30 mm). export-scene refuses part vertices deeper than `contact_tolerance_mm` (1.0, Assumption) inside any cell object | S.4 A review finding 1 |
| D9 | Export guard is a sampled check at positioner θ = 0: part surface samples (all vertices + bisection to 1.0 mm) inside each object's collision mesh; part chord 0.1 mm near joints / 0.5 mm elsewhere, object chord 0.2 mm (hull) / 0.5 mm (mesh). The true overlap can exceed `contact_tolerance_mm` (1.0, unchanged) by up to about the sample spacing plus the chords. Static objects vs the rotating part are checked only at θ = 0; other angles are Part C's scene check at θ. Recorded in `scene_manifest.json` → `export_guard` | S.4 A re-review F3 |
| D10 | Cell ↔ job link: `placement.json` does **not** hash `cell/model/scene_manifest.json` (that file carries `created_at`, so exporting job B made job A stale — against AD2). A content scene id that excludes the header links cell and job in T0.2 / A18. Face retries: accepted only if no zero-area triangle and ≥ 99 % of the B-Rep face area is covered; otherwise hull mode keeps the face nodes (`nodes_only: true`, chord null), mesh mode fails. **Open, T0.2:** `.old-<pid>` folders left by a crash are not cleaned and `Repository.jobs()` lists them; stale awm_geometry tmp files are carried forward as foreign job files; the staging pid check is wrong across host / container pid namespaces and there is no lock; an exception during rollback hides the original error | S.4 A delta review M1, M2 (lead 2026-11-03) |
