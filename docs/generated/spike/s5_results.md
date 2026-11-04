# WP-S S.5 — J2 robot-only results (H-BOX)

Record `s5_results.json`; commit `861aa590a90e0b28fdaa792996d0583cb71faeee`, dirty: false, created 2026-11-04T06:34:42Z. Scene hash `1eda0262d77350e2f919f39294d0acc2fc7e5d314527a38407804c6a951032d3`; kinematics `ur_analytic_ik-0.1.0.post3/ur5e`; scene backend `move_group/check_state_validity (cell scene v2)`; θ = 0.0° (robot-only); clearance links head_body, head_mount, conduit (F5 method: validity check (move_group check_state_validity) on unpadded geometry with the clearance links inflated by exactly the clearance (planning-scene padding diff); every other link padding 0; self-collision unpadded).

Seam S02 (J2 — solid (B2) to elbow (B3), closed butt joint, robot-only (OI-27)), length 83.786 mm, 2 arcs, overlap 2.0 mm. Scene load 4.0 s (not in C5).

| ψ case | Arc | s range (mm) | Points / pitch (mm) | C2 coverage | C3 | C5 arc (s) | Result |
|---|---|---|---|---|---|---|---|
| free | 1/2 | 0.00–43.89 | 609 / 0.072 | 0.0 % (0/609) | fail: no path: C2 fails at 609 of 609 points | 291.8 | rejected |
| free | 2/2 | 41.89–85.79 | 609 / 0.072 | 38.3 % (233/609) | fail: no path: C2 fails at 376 of 609 points | 265.1 | rejected |
| wire | 1/2 | 0.00–43.89 | 609 / 0.072 | 0.0 % (0/609) | fail: no path: C2 fails at 609 of 609 points | 160.4 | rejected |
| wire | 2/2 | 41.89–85.79 | 609 / 0.072 | 38.3 % (233/609) | fail: no path: C2 fails at 376 of 609 points | 143.1 | rejected |

| ψ case | C2 | C3 | C5 seam (s) | C8 (2 arcs) |
|---|---|---|---|---|
| free | FAIL (0.0 %, 38.3 %) | FAIL | 556.9 FAIL (≤ 30) | 0.0 % FAIL |
| wire | FAIL (0.0 %, 38.3 %) | FAIL | 303.5 FAIL (≤ 30) | 0.0 % FAIL |

## Failing points (rejected arcs; runs of consecutive points)

**free, arc 1** — no path: C2 fails at 609 of 609 points; longest continuous window 0 points (0.0 % of the arc, diagnostic only).

| Points | s (mm) | Class | Outcomes (first point of run) | Colliding / blocking pairs (run total) |
|---|---|---|---|---|
| 0–260 | 0.00–18.77 | IK | {"no_ik_unreachable": 36} |  |
| 548–608 | 39.56–43.89 | IK | {"singular_elbow": 72, "singular_shoulder": 216} |  |
| 261–547 | 18.84–39.49 | F4 | {"f4_invalid": 216, "singular_elbow": 72} | part<->nozzle_tip ×74304, part2_1<->head_body ×53960, part<->nozzle_body ×52920, part2_1<->nozzle_body ×33048 … |

**free, arc 2** — no path: C2 fails at 376 of 609 points; longest continuous window 232 points (38.0 % of the arc, diagnostic only).

| Points | s (mm) | Class | Outcomes (first point of run) | Colliding / blocking pairs (run total) |
|---|---|---|---|---|
| 0–31 | 41.89–44.13 | IK | {"no_ik_unreachable": 36} |  |
| 319–608 | 64.92–85.79 | IK | {"no_ik_unreachable": 36} |  |
| 32–85 | 44.20–48.03 | F4 | {"f4_invalid": 216, "singular_elbow": 72} | upper_arm_link<->head_mount ×7548, upper_arm_link<->head_body ×6862, shoulder_link<->head_mount ×3896, wrist_3_link<->upper_arm_link ×3457 … |

**wire, arc 1** — no path: C2 fails at 609 of 609 points; longest continuous window 0 points (0.0 % of the arc, diagnostic only).

| Points | s (mm) | Class | Outcomes (first point of run) | Colliding / blocking pairs (run total) |
|---|---|---|---|---|
| 0–260 | 0.00–18.77 | IK | {"no_ik_unreachable": 7} |  |
| 548–608 | 39.56–43.89 | IK | {"singular_elbow": 14, "singular_shoulder": 42} |  |
| 261–547 | 18.84–39.49 | F4 | {"f4_invalid": 42, "singular_elbow": 14} | part<->nozzle_tip ×14448, part2_1<->head_body ×10794, part<->nozzle_body ×10290, part2_1<->nozzle_body ×6426 … |

**wire, arc 2** — no path: C2 fails at 376 of 609 points; longest continuous window 232 points (38.0 % of the arc, diagnostic only).

| Points | s (mm) | Class | Outcomes (first point of run) | Colliding / blocking pairs (run total) |
|---|---|---|---|---|
| 0–31 | 41.89–44.13 | IK | {"no_ik_unreachable": 7} |  |
| 319–608 | 64.92–85.79 | IK | {"no_ik_unreachable": 7} |  |
| 32–85 | 44.20–48.03 | F4 | {"f4_invalid": 42, "singular_elbow": 14} | upper_arm_link<->head_mount ×1463, upper_arm_link<->head_body ×1318, shoulder_link<->head_body ×1106, shoulder_link<->head_mount ×662 … |

## Not run / open

- H-CAD head model: no head hulls exported and no H-CAD cell model (needs cad-geometry + cell-sim work; decision 2026-11-03: only if available) — open item
- C4 approach / retract / air move: S.6 / T1.5
- C8 one closed robot-only path (arcs = 1): impossible under F3 + F11 (CP-53), not run
- Mode B (positioner angles entered by the Engineer): only after the decision, B-12

## Assumptions

- part and robot base poses from CAD component placements, no REF_* markers (OI-23, scene decisions D1–D3)
- H-BOX head (TST-002 §3.1), nozzle Ø20 x 40 mm, s_c 0.3 mm, TCP at the nozzle tip, conduit at the head side (scene decision D7)
- F9 point tolerance 0.15 mm (TST-002 gives no value); F6 elbow 5° / shoulder 50 mm
- Ψ-WIRE window centre 0° (= seam X, wire-leading direction unknown, I5 / E-05) ± 15° in 5° steps
- arc_overlap_mm 2.0 (T0.5 example value; welding engineer to confirm)
- pitch floor 0.05 mm for the CP-53 refinement
- seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint), checked against the B-Rep preview frames
- F11 wrist-3 travel ±180° applied per arc (each arc is one weld segment, emission stops between arcs, CP-53)
- all holders on positioner_flange (I3 / OI-27 open)
