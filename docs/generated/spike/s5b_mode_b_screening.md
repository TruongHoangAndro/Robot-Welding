# WP-S S.5b — J2 Mode B screening (H-BOX) — positioner angle table for the Engineer

**Screening only.** The positioner angles of Mode B are Engineer-entered data (DES-001 Rev 0.6 draft §4.3 step 3, B-12). The sets at the end are *suggestions* for Truong, not a decision and not a planned path. No partial path is accepted anywhere; the final Mode B run with the chosen angles is a separate run.

Record `s5b_mode_b_screening.json`; commit `74f49e66a92e06c1dc4d7b752f476c6038b8df5f`, dirty: false, created 2026-11-04T09:53:01Z. Scene hash `1eda0262d77350e2f919f39294d0acc2fc7e5d314527a38407804c6a951032d3`; kinematics `ur_analytic_ik-0.1.0.post3/ur5e`; scene backend `move_group/check_state_validity (cell scene v2)`; head H-BOX; clearance links head_body, head_mount, conduit (F5 method: validity check (move_group check_state_validity) on unpadded geometry with the clearance links inflated by exactly the clearance (planning-scene padding diff); every other link padding 0; self-collision unpadded).

Seam S02, closed, length 83.786 mm, whole seam (not split). Pitch rule `scene_valid` (CP-53, lead decision 2026-11-04), floor 0.05 mm. Join overlap 2.0 mm. Window start stride 1.

Positioner limits (Assumption, sim cell config `src/awm_description/config/positioner_limits.yaml`, OI-27): -400° … 400°. θ step 30° → 27 angles; 12 geometric classes (θ ≡ θ ± 360°, flange FK max difference 0.00e+00 mm / 5.72e-14°). One sweep per class; the class is shown by its representative in (−180°, 180°].

## Ψ-FREE (ψ = 0 … 350°, 36 values)

| θ class (°) | In-limit θ (°) | Points / pitch (mm) | Valid-node coverage | s with ≥ 1 valid node (mm) | Longest continuous window (mm) | Dominant failure / top pairs | Time (s) |
|---|---|---|---|---|---|---|---|
| -150 | -150, 210 | 1168 / 0.072 | 16.8 % | 66.7–80.7 | 66.7–80.3 (13.6 mm, 16.2 %, S+E-W+) | IK (F4 398, IK 571, clearance 3); jig_1<->head_body ×90817, part<->nozzle_tip ×83592, part2_1<->head_body ×57592 | 634 |
| -120 | -120, 240 | 584 / 0.143 | 27.4 % | 63.7–86.5 | 63.7–86.5 (22.8 mm, 27.2 %, S-E+W+) | F4 (F4 227, IK 195, clearance 2); part<->nozzle_tip ×49032, jig_1<->head_body ×44808, part2_1<->head_body ×35336 | 394 |
| -90 | -90, 270 | 146 / 0.574 | 30.8 % | 60.8–86.1 | 60.8–86.1 (25.3 mm, 30.1 %, S-E+W+) | F4 (F4 101); jig_1<->head_body ×22112, part<->nozzle_tip ×18432, c_2500001_1<->head_body ×18016 | 140 |
| -60 | -60, 300 | 584 / 0.143 | 22.4 % | 58.2–76.9 | 59.3–76.9 (17.6 mm, 21.1 %, S+E-W+) | F4 (F4 322, IK 131); part<->nozzle_tip ×56808, c_2500001_1<->head_body ×51456, c_2500001_1<->forearm_link ×42012 | 398 |
| -30 | -390, -30, 330 | 584 / 0.143 | 18.3 % | 51.5–66.7 | 51.5–66.6 (15.1 mm, 18.0 %, S-E+W-) | IK (F4 206, IK 271); part<->nozzle_tip ×43776, c_2500001_1<->forearm_link ×33516, part2_1<->head_body ×30080 | 327 |
| 0 | -360, 0, 360 | 1168 / 0.072 | 20.1 % | 48.1–64.8 | 48.1–64.8 (16.7 mm, 19.9 %, S+E-W+) | IK (F4 343, IK 590); part<->nozzle_tip ×74664, part2_1<->head_body ×54064, part<->nozzle_body ×52992 | 546 |
| 30 | -330, 30, 390 | 1168 / 0.072 | 22.9 % | 44.9–64.1 | 44.9–64.1 (19.2 mm, 22.9 %, S-E-W-) | IK (F4 271, IK 629); part<->nozzle_tip ×77400, part2_1<->head_body ×51112, part<->nozzle_body ×50040 | 560 |
| 60 | -300, 60 | 584 / 0.143 | 29.6 % | 39.2–63.8 | 39.2–63.8 (24.7 mm, 29.5 %, S-E+W-) | IK (F4 135, IK 276); part<->nozzle_tip ×38304, part2_1<->head_body ×24608, part<->nozzle_body ×24192 | 310 |
| 90 | -270, 90 | 146 / 0.574 | 30.1 % | 39.6–64.3 | 39.6–64.3 (24.7 mm, 29.5 %, S-E-W-) | F4 (F4 102); jig_1<->head_body ×11548, part<->nozzle_tip ×9288, c_2500001_1<->head_body ×8984 | 71 |
| 120 | -240, 120 | 73 / 1.148 | 1.4 % | 60.8–60.8 | 60.8–60.8 (0.0 mm, 0.0 %, S-E+W-) | F4 (F4 42, IK 30); jig_1<->head_body ×10932, c_2500001_1<->head_body ×9696, rotary_shaft_1<->head_body ×7584 | 39 |
| 150 | -210, 150 | 73 / 1.148 | 0.0 % | none | none | IK (F4 36, IK 37); jig_1<->head_body ×9792, c_2500001_1<->head_body ×8576, c_2500001_1<->head_mount ×6336 | 37 |
| 180 | -180, 180 | 584 / 0.143 | 5.0 % | 74.9–78.9 | 74.9–78.8 (3.9 mm, 4.6 %, S-E+W-) | IK (F4 272, IK 281, clearance 2); jig_1<->head_body ×66828, c_2500001_1<->forearm_link ×41652, part<->nozzle_tip ×38088 | 262 |

### Coverage map Ψ-FREE (θ classes whose continuous windows cover each 1 mm bin)

| s (mm) | θ classes (°) |
|---|---|
| 0–2.0 | -120, -90 |
| 2–40.0 | **none** |
| 40–45.0 | 60, 90 |
| 45–49.0 | 30, 60, 90 |
| 49–52.0 | 0, 30, 60, 90 |
| 52–59.0 | -30, 0, 30, 60, 90 |
| 59–61.0 | -60, -30, 0, 30, 60, 90 |
| 61–63.0 | -90, -60, -30, 0, 30, 60, 90 |
| 63–64.0 | -90, -60, -30, 0, 30, 90 |
| 64–66.0 | -120, -90, -60, -30 |
| 66–67.0 | -120, -90, -60 |
| 67–75.0 | -150, -120, -90, -60 |
| 75–76.0 | -150, -120, -90, -60, 180 |
| 76–78.0 | -150, -120, -90, 180 |
| 78–80.0 | -150, -120, -90 |
| 80–83.8 | -120, -90 |

Bins covered by at least one θ: 54.8 %.

### Suggestions Ψ-FREE (for Truong — not a decision)

**2 angles:** no set covers 100 % with the join overlap. Best coverage: -120° / 60° → 56.5 % (θ -120°: s 63.7–86.5; θ 60°: s 39.2–63.8)

**3 angles:** no set covers 100 % with the join overlap. Best coverage: -150° / -120° / 60° → 56.5 % (θ -150°: s 66.7–80.3; θ -120°: s 63.7–86.5; θ 60°: s 39.2–63.8)

### Failing runs Ψ-FREE (per θ class; runs of consecutive points, pairs = run total)

- θ -150°: F4 s 3.2–24.5 [part<->nozzle_tip ×83592, jig_1<->head_body ×63976, part2_1<->head_body ×57592]; IK s 24.6–59.3; F4 s 59.4–66.4 [jig_1<->head_body ×26064, timing_belts_1<->head_body ×22720, c_2500001_1<->forearm_link ×12672]; clearance s 66.5–66.6 [jig_1<->head_body ×777, timing_belts_1<->head_body ×528, c_2500001_1<->forearm_link ×432]; IK s 80.8–86.9
- θ -120°: F4 s 2.9–28.0 [part<->nozzle_tip ×49032, part2_1<->head_body ×35336, part<->nozzle_body ×34920]; IK s 28.1–56.0; F4 s 56.1–63.3 [jig_1<->head_body ×13176, timing_belts_1<->head_body ×11272, c_2500001_1<->forearm_link ×9144]; clearance s 63.4–63.6 [jig_1<->head_body ×536, timing_belts_1<->head_body ×384, c_2500001_1<->forearm_link ×288]
- θ -90°: F4 s 2.9–60.3 [jig_1<->head_body ×22112, part<->nozzle_tip ×18432, c_2500001_1<->head_body ×18016]
- θ -60°: F4 s 6.5–39.0 [part<->nozzle_tip ×56808, part2_1<->head_body ×40728, part<->nozzle_body ×40104]; IK s 39.2–44.6; F4 s 44.8–58.1 [c_2500001_1<->head_body ×25584, c_2500001_1<->forearm_link ×18108, c_2500001_1<->upper_arm_link ×15048]; IK s 77.0–90.1
- θ -30°: F4 s 16.9–39.3 [part<->nozzle_tip ×43776, part2_1<->head_body ×30080, part<->nozzle_body ×29664]; IK s 39.5–44.3; F4 s 44.5–51.4 [c_2500001_1<->wrist_2_link ×14112, c_2500001_1<->head_mount ×10208, c_2500001_1<->forearm_link ×9864]; IK s 66.9–100.6
- θ 0°: F4 s 18.9–39.5 [part<->nozzle_tip ×74664, part2_1<->head_body ×54064, part<->nozzle_body ×52992]; IK s 39.6–44.1; F4 s 44.2–48.0 [upper_arm_link<->head_mount ×7572, upper_arm_link<->head_body ×6830, shoulder_link<->head_mount ×3944]; IK s 64.9–102.6
- θ 30°: F4 s 19.5–38.8 [part<->nozzle_tip ×77400, part2_1<->head_body ×51112, part<->nozzle_body ×50040]; IK s 38.9–44.8; F4 s 64.1–64.1 [forearm_link<->head_mount ×72, forearm_link<->head_body ×60, jig_1<->head_body ×32]; IK s 64.2–103.2
- θ 60°: F4 s 19.8–39.0 [part<->nozzle_tip ×38304, part2_1<->head_body ×24608, part<->nozzle_body ×24192]; IK s 64.0–103.4
- θ 90°: F4 s 64.8–122.8 [jig_1<->head_body ×11548, part<->nozzle_tip ×9288, c_2500001_1<->head_body ×8984]
- θ 120°: IK s 25.3–58.5; F4 s 59.7–59.7 [jig_1<->head_body ×60, forearm_link<->head_body ×36, timing_belts_1<->head_body ×36]; F4 s 62.0–107.9 [jig_1<->head_body ×10872, c_2500001_1<->head_body ×9696, rotary_shaft_1<->head_body ×7584]
- θ 150°: F4 s 3.4–23.0 [jig_1<->head_body ×4896, part<->nozzle_tip ×4896, timing_belts_2<->head_body ×4424]; IK s 24.1–59.7; F4 s 60.8–80.3 [jig_1<->head_body ×4896, c_2500001_1<->head_body ×4288, c_2500001_1<->head_mount ×3168]; IK s 81.5–86.1
- θ 180°: F4 s 1.9–23.5 [jig_1<->head_body ×38288, part<->nozzle_tip ×38088, timing_belts_2<->head_body ×29624]; IK s 23.7–60.3; F4 s 60.4–74.5 [jig_1<->head_body ×26592, timing_belts_1<->head_body ×17552, c_2500001_1<->forearm_link ×16308]; clearance s 74.6–74.7 [c_2500001_1<->forearm_link ×432, jig_1<->head_body ×424, c_2500001_1<->upper_arm_link ×288]; F4 s 79.1–81.9 [upper_arm_link<->head_mount ×2380, base_link_inertia<->head_body ×1952, base_link_inertia<->head_mount ×1944]; IK s 82.1–85.5

## Ψ-WIRE (ψ = -15 … 15°, 7 values)

| θ class (°) | In-limit θ (°) | Points / pitch (mm) | Valid-node coverage | s with ≥ 1 valid node (mm) | Longest continuous window (mm) | Dominant failure / top pairs | Time (s) |
|---|---|---|---|---|---|---|---|
| -150 | -150, 210 | 1168 / 0.072 | 10.7 % | 71.8–80.7 | 71.8–80.3 (8.5 mm, 10.1 %, S+E-W+) | IK (F4 469, IK 571, clearance 3); jig_1<->head_body ×20815, part<->nozzle_tip ×16254, timing_belts_2<->head_body ×11510 | 122 |
| -120 | -120, 240 | 584 / 0.143 | 23.8 % | 66.7–86.5 | 66.7–86.5 (19.8 mm, 23.6 %, S-E+W-) | F4 (F4 248, IK 195, clearance 2); part<->nozzle_tip ×9534, jig_1<->head_body ×9029, part2_1<->head_body ×7014 | 87 |
| -90 | -90, 270 | 146 / 0.574 | 30.8 % | 60.8–86.1 | 60.8–86.1 (25.3 mm, 30.1 %, S-E+W+) | F4 (F4 101); c_2500001_1<->head_body ×3640, part<->nozzle_tip ×3584, jig_1<->head_body ×3448 | 30 |
| -60 | -60, 300 | 584 / 0.143 | 22.4 % | 58.2–76.9 | 58.2–75.3 (17.1 mm, 20.4 %, S-E+W-) | F4 (F4 322, IK 131); part<->nozzle_tip ×11046, c_2500001_1<->head_body ×10448, c_2500001_1<->forearm_link ×8169 | 105 |
| -30 | -390, -30, 330 | 584 / 0.143 | 18.3 % | 51.5–66.7 | 51.5–66.6 (15.1 mm, 18.0 %, S-E+W-) | IK (F4 206, IK 271); part<->nozzle_tip ×8512, c_2500001_1<->forearm_link ×6517, part2_1<->head_body ×5992 | 74 |
| 0 | -360, 0, 360 | 1168 / 0.072 | 20.1 % | 48.1–64.8 | 48.1–64.8 (16.7 mm, 19.9 %, S+E-W+) | IK (F4 343, IK 590); part<->nozzle_tip ×14518, part2_1<->head_body ×10808, part<->nozzle_body ×10304 | 161 |
| 30 | -330, 30, 390 | 1168 / 0.072 | 22.9 % | 44.9–64.1 | 44.9–64.1 (19.2 mm, 22.9 %, S-E-W-) | IK (F4 271, IK 629); part<->nozzle_tip ×15050, part2_1<->head_body ×10234, part<->nozzle_body ×9730 | 219 |
| 60 | -300, 60 | 292 / 0.287 | 25.3 % | 39.3–60.3 | 39.3–60.3 (20.9 mm, 25.0 %, S-E+W+) | IK (F4 79, IK 138, clearance 1); part<->nozzle_tip ×3738, part2_1<->head_body ×2450, part<->nozzle_body ×2338 | 39 |
| 90 | -270, 90 | 146 / 0.574 | 21.9 % | 39.6–57.4 | 39.6–57.4 (17.8 mm, 21.2 %, S-E-W-) | F4 (F4 114); jig_1<->head_body ×2242, c_2500001_1<->head_body ×1820, part<->nozzle_tip ×1806 | 19 |
| 120 | -240, 120 | 73 / 1.148 | 0.0 % | none | none | F4 (F4 43, IK 30); jig_1<->head_body ×2156, c_2500001_1<->head_body ×1944, rotary_shaft_1<->head_body ×1872 | 11 |
| 150 | -210, 150 | 73 / 1.148 | 0.0 % | none | none | IK (F4 36, IK 37); jig_1<->head_body ×1904, c_2500001_1<->head_body ×1488, c_2500001_1<->forearm_link ×1183 | 10 |
| 180 | -180, 180 | 73 / 1.148 | 0.0 % | none | none | F4 (F4 38, IK 35); jig_1<->head_body ×1636, c_2500001_1<->forearm_link ×1141, part<->nozzle_tip ×924 | 9 |

### Coverage map Ψ-WIRE (θ classes whose continuous windows cover each 1 mm bin)

| s (mm) | θ classes (°) |
|---|---|
| 0–2.0 | -120, -90 |
| 2–40.0 | **none** |
| 40–45.0 | 60, 90 |
| 45–49.0 | 30, 60, 90 |
| 49–52.0 | 0, 30, 60, 90 |
| 52–57.0 | -30, 0, 30, 60, 90 |
| 57–59.0 | -30, 0, 30, 60 |
| 59–60.0 | -60, -30, 0, 30, 60 |
| 60–61.0 | -60, -30, 0, 30 |
| 61–64.0 | -90, -60, -30, 0, 30 |
| 64–66.0 | -90, -60, -30 |
| 66–67.0 | -90, -60 |
| 67–72.0 | -120, -90, -60 |
| 72–76.0 | -150, -120, -90, -60 |
| 76–80.0 | -150, -120, -90 |
| 80–83.8 | -120, -90 |

Bins covered by at least one θ: 54.8 %.

### Suggestions Ψ-WIRE (for Truong — not a decision)

**2 angles:** no set covers 100 % with the join overlap. Best coverage: -90° / 60° → 55.1 % (θ -90°: s 60.8–86.1; θ 60°: s 39.3–60.3)

**3 angles:** no set covers 100 % with the join overlap. Best coverage: -120° / -60° / 60° → 56.3 % (θ -120°: s 66.7–86.5; θ -60°: s 58.2–75.3; θ 60°: s 39.3–60.3)

### Failing runs Ψ-WIRE (per θ class; runs of consecutive points, pairs = run total)

- θ -150°: F4 s 3.2–24.5 [part<->nozzle_tip ×16254, jig_1<->head_body ×11638, timing_belts_2<->head_body ×11510]; IK s 24.6–59.3; F4 s 59.4–71.5 [jig_1<->head_body ×9044, timing_belts_1<->head_body ×7412, c_2500001_1<->forearm_link ×4452]; clearance s 71.6–71.7 [jig_1<->head_body ×133, c_2500001_1<->forearm_link ×84, c_2500001_1<->upper_arm_link ×84]; IK s 80.8–86.9
- θ -120°: F4 s 2.9–28.0 [part<->nozzle_tip ×9534, part2_1<->head_body ×7014, part<->nozzle_body ×6790]; IK s 28.1–56.0; F4 s 56.1–66.3 [jig_1<->head_body ×3738, timing_belts_1<->head_body ×2682, c_60cm30x_3d_1<->head_body ×2490]; clearance s 66.4–66.6 [jig_1<->head_body ×85, c_2500001_1<->forearm_link ×42, c_2500001_1<->upper_arm_link ×42]
- θ -90°: F4 s 2.9–60.3 [c_2500001_1<->head_body ×3640, part<->nozzle_tip ×3584, jig_1<->head_body ×3448]
- θ -60°: F4 s 6.5–39.0 [part<->nozzle_tip ×11046, part2_1<->head_body ×8134, part<->nozzle_body ×7798]; IK s 39.2–44.6; F4 s 44.8–58.1 [c_2500001_1<->head_body ×5200, c_2500001_1<->forearm_link ×3521, c_2500001_1<->upper_arm_link ×2926]; IK s 77.0–90.1
- θ -30°: F4 s 16.9–39.3 [part<->nozzle_tip ×8512, part2_1<->head_body ×5992, part<->nozzle_body ×5768]; IK s 39.5–44.3; F4 s 44.5–51.4 [c_2500001_1<->wrist_2_link ×2744, c_2500001_1<->forearm_link ×1918, c_2500001_1<->head_mount ×1784]; IK s 66.9–100.6
- θ 0°: F4 s 18.9–39.5 [part<->nozzle_tip ×14518, part2_1<->head_body ×10808, part<->nozzle_body ×10304]; IK s 39.6–44.1; F4 s 44.2–48.0 [upper_arm_link<->head_mount ×1468, upper_arm_link<->head_body ×1314, shoulder_link<->head_body ×1106]; IK s 64.9–102.6
- θ 30°: F4 s 19.5–38.8 [part<->nozzle_tip ×15050, part2_1<->head_body ×10234, part<->nozzle_body ×9730]; IK s 38.9–44.8; F4 s 64.1–64.1 [forearm_link<->head_mount ×14, forearm_link<->head_body ×10, c_2500001_1<->conduit ×2]; IK s 64.2–103.2
- θ 60°: F4 s 19.8–39.0 [part<->nozzle_tip ×3738, part2_1<->head_body ×2450, part<->nozzle_body ×2338]; clearance s 60.5–60.5 [c_2500001_1<->conduit ×53, forearm_link<->head_body ×22, forearm_link<->head_mount ×21]; F4 s 60.8–63.7 [c_2500001_1<->conduit ×560, forearm_link<->head_body ×263, forearm_link<->head_mount ×224]; IK s 64.0–103.3
- θ 90°: F4 s 58.0–122.8 [jig_1<->head_body ×2242, c_2500001_1<->head_body ×1820, part<->nozzle_tip ×1806]
- θ 120°: IK s 25.3–58.5; F4 s 59.7–107.9 [jig_1<->head_body ×2156, c_2500001_1<->head_body ×1944, rotary_shaft_1<->head_body ×1872]
- θ 150°: F4 s 3.4–23.0 [jig_1<->head_body ×952, part<->nozzle_tip ×952, timing_belts_2<->head_body ×952]; IK s 24.1–59.7; F4 s 60.8–80.3 [jig_1<->head_body ×952, c_2500001_1<->head_body ×744, timing_belts_1<->head_body ×672]; IK s 81.5–86.1
- θ 180°: F4 s 2.3–23.0 [part<->nozzle_tip ×924, jig_1<->head_body ×920, timing_belts_2<->head_body ×868]; IK s 24.1–59.7; F4 s 60.8–81.5 [jig_1<->head_body ×716, c_2500001_1<->forearm_link ×567, timing_belts_1<->head_body ×532]; IK s 82.6–84.9

## Not run / open

- final Mode B run with the angles chosen by the Engineer (next run, after Truong picks 2–3 angles)
- index moves between angles, approach / retract / air moves (C4): not part of the screening
- H-CAD head model: no head hulls / H-CAD cell model (open item, as in S.5)

## Assumptions

- positioner limits -400…400° from the sim cell config (awm_description positioner_limits.yaml), not the real drive / cables / belts (OI-14, OI-27)
- θ ≡ θ ± 360°: the scene is periodic in θ (checked by flange FK); cable wrap of the real positioner not modelled (OI-27)
- all holders on positioner_flange, i.e. they rotate with the part (I3 / OI-27 open)
- part and robot base poses from CAD component placements, no REF_* markers (OI-23, scene decisions D1–D3)
- H-BOX head (TST-002 §3.1), nozzle Ø20 x 40 mm, s_c 0.3 mm, TCP at the nozzle tip, conduit at the head side (scene decision D7)
- F9 point tolerance 0.15 mm (TST-002 gives no value); F6 elbow 5° / shoulder 50 mm
- Ψ-WIRE window centre 0° (= seam X, wire-leading direction unknown, I5 / E-05) ± 15° in 5° steps
- join overlap 2.0 mm between Mode B segments = seam arc_overlap_mm (T0.5 example value; welding engineer to confirm, OI-11 / OI-16)
- pitch floor 0.05 mm; CP-53 pitch rule on scene-valid nodes (lead decision 2026-11-04)
- F11 wrist-3 travel ±180° applied per window (each Mode B segment is one weld segment, emission stops between segments, DES-001 §6.2)
- B-03 margin 3.0 mm (S.6 Assumption; = F4 padding) — recorded, not used: no positioner index move is planned in the screening
- covering sets use one continuous window per angle (one weld segment per angle); joins need the overlap
- seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint), S.5 record
