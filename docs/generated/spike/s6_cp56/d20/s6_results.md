# WP-S S.6 — J1 Mode C clearance check (H-BOX) — TST-002 C9 (reported, does not gate)

Record `s6_results.json`; commit `8cc8cb660fc22639cca1c905515b8971a123b588`, dirty: false, created 2026-11-05T15:52:04Z. Scene hash `1eda0262d77350e2f919f39294d0acc2fc7e5d314527a38407804c6a951032d3`; kinematics `ur_analytic_ik-0.1.0.post3/ur5e`; scene backend `move_group/check_state_validity (cell scene v2)`; clearance links head_body, head_mount, conduit (F5 method: validity check (move_group check_state_validity) on unpadded geometry with the clearance links inflated by exactly the clearance (planning-scene padding diff); every other link padding 0; self-collision unpadded).

Head H-BOX, head args {"nozzle_dia_mm": "20"}: nozzle Ø20 mm, s_c 0.3 mm, tool0_T_tcp [0.0, 0.0, 481.0] mm (quat [0.0, 0.0, 0.0, 1.0]) — read from the robot description `79391356a49e203c…` and checked against /compute_fk. F9 rule (b) **rising**, point / plane tolerance 0.15 mm.

Seam S01 (J1 — pipe nipple (B1) to solid (B2), closed butt joint, Mode C (coaxial with the positioner axis)), r 13.335 mm, length 83.786 mm, closing overlap 2.0 mm = 8.59°; span 368.59°.

- F8 coaxiality (seam circle vs positioner axis, flange frame): centre offset 0.0021 mm, axis angle 0.0000° → PASS (tol 0.05 mm / 0.1°).
- Rotating set r_max 130.05 mm (part3_1); B-03 step Δθ_max = 2·(5.0 − 3.0) / r_max = 1.762° → 211 angles at 1.755°.
- Flange model check (world_T_flange(0)·Rx(θ) vs /compute_fk): max 0.00e+00 mm / 9.21e-15° at θ [90.0, -180.0, -368.0, 270.0].
- F9 per θ: the TCP is fixed in world; the contact check runs on part_T_tcp(θ) = (world_T_flange(θ)·flange_T_part)⁻¹·world_T_tcp, i.e. on the seam point that is under the nozzle at that angle (s advances r·|θ|). For a seam within F8 and a nozzle that is a body of revolution about the beam axis, the set of nozzle poses relative to the part over one full turn is the same for every Mode C robot pose on the circle and every ψ (only the θ at which each seam point is reached changes): the F9 verdict over the turn does not depend on the chosen pose; F4 / F5 do.

## Pose choice at θ = 0

| ψ case | Points facing the robot / sampled | Candidates | Kinematically valid | F4 valid | Clearance ≥ 5 mm | F9 points ok | Valid nodes | Chosen (point, s, ψ, label, cost) |
|---|---|---|---|---|---|---|---|---|
| free | 17 / 36 | 3600 | 2448 | 396 | 366 | 17 / 17 | 366 | 10, s 23.27, ψ 0°, S+E-W-, 0.426 |
| wire | 17 / 36 | 700 | 476 | 100 | 100 | 17 / 17 | 100 | 10, s 23.27, ψ -15°, S+E-W-, 0.426 |

## θ sweep — free, rank 1 (result): point 10 (s 23.27 mm), ψ 0°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 594 angles) {"F10": 0.014, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.086, "F9": 0.049}; failing ranges 0: 

## θ sweep — free, rank 94 (diagnostic): point 9 (s 20.95 mm), ψ 0°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 594 angles) {"F10": 0.014, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.087, "F9": 0.049}; failing ranges 0: 

## θ sweep — free, rank 203 (diagnostic): point 8 (s 18.62 mm), ψ 0°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 598 angles) {"F10": 0.015, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.087, "F9": 0.049}; failing ranges 0: 

## θ sweep — wire, rank 1 (result): point 10 (s 23.27 mm), ψ -15°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 594 angles) {"F10": 0.014, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.086, "F9": 0.049}; failing ranges 0: 

## θ sweep — wire, rank 29 (diagnostic): point 9 (s 20.95 mm), ψ -15°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 594 angles) {"F10": 0.014, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.087, "F9": 0.049}; failing ranges 0: 

## θ sweep — wire, rank 60 (diagnostic): point 8 (s 18.62 mm), ψ -15°, S+E-W-, θ sign -1

C9 **PASS** — 211 / 211 angles ok; min clearance lower bound 20.0 mm at θ 0.0, -1.8, -3.5, -5.3, -7.0, -8.8 … (closest pairs {"jig_1<->head_body": 211}); guaranteed between checked angles 18.01 mm.

| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not passed) | F9 |
|---|---|---|---|---|---|---|---|
| 0.00 → -368.59 | 211 | yes | valid |  | 20.0 | jig_1<->head_body | ok |

F9 fine profile (0.499° step, 739 angles): 0 fail (0.0 %), reasons {}; offending faces (min distance mm) {}; excused by rule (b) (within s_c of the disc / rim, not rising above the contact plane; 598 angles) {"F10": 0.015, "F16": 0.016, "F17": 0.052, "F18": 0.019, "F19": 0.016, "F8": 0.087, "F9": 0.049}; failing ranges 0: 

## Not run / open

- H-CAD head model: S.5c Task 2 / 3 (H-CAD description not built yet)
- approach / retract / air move to the Mode C pose (C4): T1.5
- time parameterisation ω = travel speed / r and ramps in the overlap (A47, B-20): WPS travel speed not given (I5); the check is geometric (quasi-static, positioner angles only)
- TST-002 §11 not filled (S.7, lead)

## Assumptions

- part, robot base and positioner axis poses from CAD component placements, no REF_* markers (OI-23)
- H-BOX head (TST-002 §3.1), nozzle Ø20 mm flat disc, s_c 0.3 mm, TCP at the nozzle tip (I2 / OI-15); nozzle, s_c and tool0_T_tcp read from the robot description in use (S.5c Task 0, CP-56 one source)
- F9 point tolerance 0.15 mm (TST-002 gives no value); rule (b) 'rising' (CP-56: only faces rising above the contact plane by more than the tolerance count; the tolerance is also the plane tolerance)
- B-03 angular step: margin 3.0 mm (= F4 padding_mm; TST-002 / DES-001 give no margin value), c = weld clearance verified at every checked angle; between checked angles only c − r_max·Δθ/2 is guaranteed
- r_max over every object on positioner_flange (all holders rotate with the part; I3 / OI-27 open)
- positioner flange turns about its own X axis (frames.yaml), checked against /compute_fk
- closing overlap 2.0 mm (D04 arc_overlap_mm of the Mode C seam, T0.5 example value; welding engineer to confirm)
- contact location: seam points at the data-file step whose seam Z faces the robot base origin (cos ≥ 0.0); Ψ-WIRE window centre 0° = seam X (I5)
- seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint)
