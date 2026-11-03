# Cell CAD component tree — summary (WP-S S.4, task A1)

| Item | Value |
|---|---|
| Date | 2026-11-03 |
| Source | `docs/3D/APLSMN601_Automated Weldment System.step` (Fusion export), SHA-256 `bc66c09ef1878d0e7389cf852b695d8b59917994a98d5739e67ba627bab7e824` |
| Command | `$AWM_OCC_PYTHON -m awm_geometry.cli inspect-assembly --step <cell STEP> --out .worktrees/spike_repo/cell_components.json` (63 s) |
| Tool | awm_geometry 0.1.0, pythonocc 8.0.1 |
| Full output | `.worktrees/spike_repo/cell_components.json` (git-ignored spike repository; 269 components, 388 solids) |
| Frame | **File frame, mm, Y up** (no rotation to world Z-up applied). Placement = 4×4 of the component origin in the file frame; bbox = exact B-Rep bounds of the placed solids |
| Unit declared in the file | mm |

Purpose: input for the role map `docs/generated/spike/cad_map_cell.yaml` (task A2, written by the lead). The role
marks below are the agent's reading of names and geometry — **Assumption** until the mechanical team confirms them
(no `REF_*` marker components in this export, OI-23).

## 1. Component tree, depth 1 and 2

Root (depth 0): `APLSMN601_Automated Weldment System`, 388 solids, bbox X −562.5 … 384.1, Y −495.6 … 1259.1,
Z −499.1 … 657.1 mm. Paths are the instance names joined by ` / `. Note: the UR product name `UR5E/UR7E` contains
a slash, so map entries should match on the instance / product name, not split the path string on `/`.

| # | Depth | Path (below the root) | Instance | Product | X min … max | Y min … max | Z min … max | Solids | Origin (mm) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 2500001:1 | `2500001:1` | `2500001` | -562.5 … 384.1 | -495.6 … 429.4 | -371.3 … 563.7 | 321 | 0.00, 0.00, 0.00 |
| 2 | 2 | 2500001:1 / 2600001:1 | `2600001:1` | `2600001` | -562.5 … 384.1 | -495.6 … 418.4 | -371.3 … 563.7 | 317 | -100.03, 3.40, 96.58 |
| 226 | 2 | 2500001:1 / SOCKET HEAD CAP SCREW (4):1 | `SOCKET HEAD CAP SCREW (4):1` | `SOCKET HEAD CAP SCREW (4)` | 331.0 … 344.0 | 401.4 … 429.4 | -349.9 … -336.9 | 1 | 337.47, 415.40, -343.42 |
| 227 | 2 | 2500001:1 / SOCKET HEAD CAP SCREW (4):2 | `SOCKET HEAD CAP SCREW (4):2` | `SOCKET HEAD CAP SCREW (4)` | -544.0 … -531.0 | 401.4 … 429.4 | -349.9 … -336.9 | 1 | -537.53, 415.40, -343.42 |
| 228 | 2 | 2500001:1 / SOCKET HEAD CAP SCREW (4):3 | `SOCKET HEAD CAP SCREW (4):3` | `SOCKET HEAD CAP SCREW (4)` | 331.0 … 344.0 | 401.4 … 429.4 | 530.1 … 543.1 | 1 | 337.47, 415.40, 536.58 |
| 229 | 2 | 2500001:1 / SOCKET HEAD CAP SCREW (4):4 | `SOCKET HEAD CAP SCREW (4):4` | `SOCKET HEAD CAP SCREW (4)` | -544.0 … -531.0 | 401.4 … 429.4 | 530.1 … 543.1 | 1 | -537.53, 415.40, 536.58 |
| 230 | 1 | 564082-Assembly-3 parts:2 | `564082-Assembly-3 parts:2` | `564082-Assembly-3 parts` | -235.5 … -63.9 | 580.1 … 721.2 | 308.2 … 346.7 | 3 | -102.03, 593.40, 321.58 |
| 231 | 2 | 564082-Assembly-3 parts:2 / 296783_PIPE NIPPLE:1 | `296783_PIPE NIPPLE:1` | `296783_PIPE NIPPLE` | -235.5 … -210.1 | 580.1 … 606.7 | 308.2 … 334.9 | 1 | -222.83, 593.40, 321.58 |
| 232 | 2 | 564082-Assembly-3 parts:2 / 564082-Solid:1 | `564082-Solid:1` | `564082-Solid` | -210.1 … -88.7 | 580.1 … 671.3 | 308.2 … 342.7 | 1 | -102.03, 593.40, 321.58 |
| 233 | 2 | 564082-Assembly-3 parts:2 / 564082-Elbow:1 | `564082-Elbow:1` | `564082-Elbow` | -115.4 … -63.9 | 668.6 … 721.2 | 316.2 … 346.7 | 1 | -102.03, 593.40, 321.58 |
| 234 | 1 | Jig:1 | `Jig:1` | `Jig` | -267.0 … -20.0 | 418.4 … 613.4 | 246.6 … 396.6 | 1 | -143.53, 568.40, 246.58 |
| 235 | 1 | Part1:1 | `Part1:1` | `Part1` | -290.0 … -223.5 | 578.4 … 608.4 | 306.6 … 336.6 | 1 | -290.03, 593.40, 321.58 |
| 236 | 1 | HTPK20S5M150-A-C15:1 | `HTPK20S5M150-A-C15:1` | `HTPK20S5M150-A-C15` | -290.0 … -268.0 | 575.4 … 611.4 | 303.6 … 339.6 | 5 | -268.03, 593.40, 321.58 |
| 237 | 1 | HTPK20S5M150-A-C15:2 | `HTPK20S5M150-A-C15:2` | `HTPK20S5M150-A-C15` | -290.0 … -268.0 | 430.4 … 466.4 | 303.6 … 339.6 | 5 | -268.03, 448.40, 321.58 |
| 238 | 1 | Timing Belts:1 | `Timing Belts:1` | `Timing Belts` | -286.5 … -271.5 | 431.4 … 610.4 | 304.6 … 338.6 | 1 | -279.03, 520.90, 321.58 |
| 239 | 1 | Rotary Shaft:1 | `Rotary Shaft:1` | `Rotary Shaft` | -304.6 … -13.1 | 442.4 … 454.4 | 315.6 … 327.6 | 1 | -13.13, 448.40, 321.58 |
| 240 | 1 | 60CM30X_3D:1 | `60CM30X_3D:1` | `60CM30X_3D` | -417.1 … -310.6 | 418.3 … 478.5 | 291.5 … 351.7 | 1 | -404.63, 448.40, 321.58 |
| 241 | 1 | GSACL25-8-12:1 | `GSACL25-8-12:1` | `GSACL25-8-12` | -324.0 … -293.0 | 435.9 … 460.9 | 309.1 … 334.1 | 3 | -324.03, 448.40, 321.58 |
| 242 | 2 | GSACL25-8-12:1 / GSACL25-8-12_b:1 | `GSACL25-8-12_b:1` | `GSACL25-8-12_b` | -324.0 … -293.0 | 435.9 … 460.9 | 309.1 … 334.1 | 1 | -324.03, 448.40, 321.58 |
| 243 | 2 | GSACL25-8-12:1 / BOLT3-10:1 | `BOLT3-10:1` | `BOLT3-10` | -299.8 … -294.3 | 439.9 … 453.3 | 327.4 … 333.6 | 1 | -297.03, 449.93, 330.46 |
| 244 | 2 | GSACL25-8-12:1 / BOLT3-10:2 | `BOLT3-10:2` | `BOLT3-10` | -322.8 … -317.3 | 436.4 … 442.6 | 313.1 … 326.5 | 1 | -320.03, 439.52, 323.11 |
| 245 | 1 | Part2:1 | `Part2:1` | `Part2` | -48.0 … -38.0 | 582.5 … 718.7 | 308.4 … 346.5 | 1 | -38.03, 650.63, 327.45 |
| 246 | 1 | Part3:1 | `Part3:1` | `Part3` | -77.6 … -38.0 | 692.9 … 722.9 | 318.3 … 348.3 | 1 | -38.03, 707.86, 333.33 |
| 247 | 1 | HTPK20S5M150-A-C15:3 | `HTPK20S5M150-A-C15:3` | `HTPK20S5M150-A-C15` | -18.0 … 4.0 | 430.4 … 466.4 | 303.6 … 339.6 | 5 | 3.97, 448.40, 321.58 |
| 248 | 1 | HTPK20S5M150-A-C15:4 | `HTPK20S5M150-A-C15:4` | `HTPK20S5M150-A-C15` | -18.0 … 4.0 | 575.4 … 611.4 | 303.6 … 339.6 | 5 | 3.97, 593.40, 321.58 |
| 249 | 1 | Timing Belts:2 | `Timing Belts:2` | `Timing Belts` | -14.5 … 0.5 | 431.4 … 610.4 | 304.6 … 338.6 | 1 | -7.03, 520.90, 321.58 |
| 250 | 1 | UR5E/UR7E:1 | `UR5E/UR7E:1` | `UR5E/UR7E` | -309.9 … -24.5 | 513.5 … 1259.1 | -499.1 … 657.1 | 29 | -100.03, 518.40, -153.42 |
| 251 | 2 | UR5E/UR7E:1 / C-1000248:1 | `C-1000248:1` | `C-1000248` | -174.4 … -39.9 | 617.5 … 752.9 | -213.5 … -93.3 | 1 | -100.03, 518.40, -153.42 |
| 252 | 2 | UR5E/UR7E:1 / C-1000249:1 | `C-1000249:1` | `C-1000249` | -309.9 … -174.4 | 620.8 … 1151.5 | -213.5 … 16.7 | 1 | -100.03, 1248.38, -1.37 |
| 253 | 2 | UR5E/UR7E:1 / C-1000250:1 | `C-1000250:1` | `C-1000250` | -174.4 … -49.3 | 917.9 … 1149.4 | -101.4 … 364.5 | 1 | -106.63, 1091.42, -43.42 |
| 254 | 2 | UR5E/UR7E:1 / C-1000251:1 | `C-1000251:1` | `C-1000251` | -272.7 … -152.9 | 902.7 … 1011.9 | 262.3 … 383.5 | 3 | -100.03, 460.74, 473.48 |
| 255 | 2 | UR5E/UR7E:1 / C-2007038:1 | `C-2007038:1` | `C-2007038` | -273.7 … -195.8 | 901.1 … 979.9 | 401.8 … 488.7 | 1 | -233.33, 915.17, 459.61 |
| 256 | 2 | UR5E/UR7E:1 / C-2007861:1 | `C-2007861:1` | `C-2007861` | -272.7 … -194.0 | 945.5 … 1065.9 | 348.8 … 467.9 | 1 | -233.33, 1003.45, 413.49 |
| 257 | 2 | UR5E/UR7E:1 / C-1000257:1 | `C-1000257:1` | `C-1000257` | -175.5 … -24.5 | 513.5 … 617.5 | -499.1 … -77.9 | 19 | -100.03, 518.40, -153.42 |
| 258 | 2 | UR5E/UR7E:1 / ASSY, Welding tool:1 | `ASSY, Welding tool:1` | `ASSY, Welding tool` | -268.3 … -198.3 | 648.3 … 1259.1 | 345.4 … 657.1 | 2 | -233.33, 835.78, 432.23 |
| 263 | 1 | ASSY, Welding tool:1 | `ASSY, Welding tool:1` | `ASSY, Welding tool` | -268.3 … -198.3 | 648.3 … 1259.1 | 345.4 … 657.1 | 2 | -233.33, 835.78, 432.23 |
| 264 | 2 | ASSY, Welding tool:1 / LightWELD:1 | `LightWELD:1` | `LightWELD` | -252.8 … -213.8 | 648.3 … 1259.1 | 345.4 … 657.1 | 1 | -233.33, 776.64, 632.53 |
| 267 | 2 | ASSY, Welding tool:1 / Tool for LightWELD:1 | `Tool for LightWELD:1` | `Tool for LightWELD` | -268.3 … -198.3 | 793.6 … 933.6 | 427.7 … 597.8 | 1 | -233.33, 917.83, 458.22 |
| 268 | 1 | Part5:1 | `Part5:1` | `Part5` | -180.0 … -20.0 | 418.4 … 518.4 | -233.4 … -73.4 | 2 | -100.03, 508.40, -153.42 |

### Lower frame `2500001:1 / 2600001:1` (depth 3, without fasteners)

| # | Instance | X min … max | Y min … max | Z min … max | Solids |
|---|---|---|---|---|---|
| 3 | `1000000488:1` | -562.5 … -482.5 | -411.6 … 418.4 | -365.9 … 559.1 | 22 |
| 4 | `1000000489:1` | -559.5 … 359.5 | -411.6 … -386.6 | -362.9 … 556.1 | 27 |
| 5 | `2600002:1` | -481.5 … 281.5 | -406.7 … 415.1 | 544.2 … 563.7 | 12 |
| 18 | `2600003:1` | -481.5 … 281.5 | -406.7 … 415.1 | -371.3 … -334.9 | 27 |
| 38 | `2600005:1` | -559.5 … -469.5 | 291.0 … 404.0 | -362.9 … -272.9 | 8 |
| 44 | `1000000490:1` | -482.5 … 282.5 | -410.0 … 418.4 | 532.1 … 557.1 | 7 |
| 45 | `HRP-POA_50G_757019:1` | 284.0 … 384.1 | -495.6 … -411.6 | -335.4 … -262.4 | 1 |
| 55 | `1000000495:1` | -500.0 … 300.0 | 393.0 … 418.4 | -303.4 … 496.6 | 1 |
| 56 | `1000000488:2` | 282.5 … 362.5 | -411.6 … 418.4 | -365.9 … 559.1 | 22 |
| 57 | `2600005:2` | 269.5 … 359.5 | 291.0 … 404.0 | 466.1 … 556.1 | 8 |
| 63 | `1000000490:2` | -482.5 … 282.5 | -410.0 … 418.4 | -363.9 … -338.9 | 7 |
| 64 | `2600005:3` | 269.5 … 359.5 | 291.0 … 404.0 | -362.9 … -272.9 | 8 |
| 70 | `2600005:4` | -559.5 … -469.5 | 291.0 … 404.0 | 466.1 … 556.1 | 8 |
| 76 | `HRP-POA_50G_757019:2` | 284.0 … 384.1 | -495.6 … -411.6 | 455.6 … 528.6 | 1 |
| 77 | `HRP-POA_50G_757019:3` | -557.0 … -456.9 | -495.6 … -411.6 | 455.6 … 528.6 | 1 |
| 78 | `HRP-POA_50G_757019:4` | -557.0 … -456.9 | -495.6 … -411.6 | -335.4 … -262.4 | 1 |

Fasteners under `2600001:1` (156 leaf parts, one solid each): FLAT Washer × 16, FLAT Washer (1) × 16,
Hexagon Nut (1) × 16, Hexagon Nut (2) × 16, Plain Washer × 16, SOCKET HEAD CAP SCREW (1) × 12,
SOCKET HEAD CAP SCREW (2) × 16, Spring Washer × 16, Spring Washer (1) × 32.

## 2. What each component looks like (role candidates — Assumption)

| Looks like | Component(s) | Evidence |
|---|---|---|
| Lower enclosure frame + Pismo plate | `2500001:1` (all of it) | Frame profiles `1000000488` / `1000000489` / `1000000490`, beams `2600002` / `2600003`, brackets `2600005` ×4, levelling feet `HRP-POA_50G_757019` ×4 at Y −495.6 … −411.6 (floor). **Pismo plate = `1000000495:1`**: 800 × 800 × 25.4 mm, top face at **Y = 418.40** |
| UR5e pedestal / adapter | `Part5:1` | 160 × 160 × 100 mm block on the plate (Y 418.40 … 518.40), centred under the UR base (X −100.03, Z −153.42) |
| UR5e | `UR5E/UR7E:1` (29 solids incl. the welding tool) | Links `C-1000248` … `C-1000257`, `C-2007038`, `C-2007861` (UR part numbers). `C-1000257` (19 solids) = base: bottom at Y 513.5. The arm is not at zero joints in the CAD (pose not used: the model comes from `ur_description`, ADR-16) |
| LightWELD head + mount | `UR5E/UR7E:1 / ASSY, Welding tool:1` **and** top-level `ASSY, Welding tool:1` | Children `LightWELD:1` (head, 1 solid, Y 648.3 … 1259.1) and `Tool for LightWELD:1` (mount, cylinder r 15 mm, axis (0, 0.886, −0.463) = 27.6° from file Y). **Duplicate:** both instances have the identical placement and bbox — the same tool is in the file twice (OI-23). Only one may be mapped (the other `ignore`), otherwise the collision scene holds the head twice |
| Jig (base / bearing frame of the positioner) | `Jig:1` | 1 solid, X −267.0 … −20.0, Y 418.4 … 613.4 (stands on the plate), Z 246.6 … 396.6; its largest cylinder is a Ø12 bore on the drive-shaft axis (Y 448.40, Z 321.58) |
| Holders | `Part1:1`, `Part2:1`, `Part3:1` | Part1: X −290.0 … −223.5, Ø12 cylinder **on the part axis** (Y 593.40, Z 321.58), holds the pipe-nipple end. Part2: X −48.0 … −38.0, plate Y 582.5 … 718.7, Ø12 cylinder on the part axis. Part3: X −77.6 … −38.0, Ø30 cylinder on the elbow-end axis (Y 707.86, Z 333.33), holds the elbow. Which ones rotate is open (OI-27) |
| Rotary (drive) shaft | `Rotary Shaft:1` | Ø12 mm, X −304.6 … −13.1, axis on Y 448.40, Z 321.58 (see §3) |
| Motor | `60CM30X_3D:1` | X −417.1 … −310.6, origin on the drive-shaft axis (Y 448.40, Z 321.58) |
| Coupling | `GSACL25-8-12:1` (+ 2 × `BOLT3-10`) | X −324.0 … −293.0, between motor and shaft, on the drive-shaft axis |
| Belt pulleys | `HTPK20S5M150-A-C15:1 … :4` (5 solids each) | :2 and :3 on the drive shaft (Y 448.40) at X ≈ −279 and ≈ −7; :1 and :4 on the **part axis** (Y 593.40) at the same X |
| Timing belts | `Timing Belts:1`, `Timing Belts:2` | Y 431.4 … 610.4 at X ≈ −279 and ≈ −7: two belt stages, one at each end, drive shaft → part axis (centre distance 145.0 mm) |
| Golden part | `564082-Assembly-3 parts:2` | Instance suffix `:2` (only one instance in the file; match on the product name). 3 solids: `296783_PIPE NIPPLE:1`, `564082-Solid:1`, `564082-Elbow:1` |
| Fasteners | `SOCKET HEAD CAP SCREW (4):1 … :4` under `2500001:1`, 156 fasteners under `2600001:1` | Static, belong to the frame |

Not in the file: upper laser enclosure (OI-24), `REF_*` marker components (OI-23).

## 3. Key placements (file frame, mm, Y up)

**UR5e** `UR5E/UR7E:1` — rotation = identity, origin **(−100.03, 518.40, −153.42)**. The origin lies on the
top face of the pedestal `Part5` (Y 518.40) at the pedestal centre, so it is the robot mounting point; the UR model's
own up axis is the file Y axis. Whether this origin equals `ur_description` `base_link` (and its rotation about the
up axis) is an **Assumption** until `REF_UR_BASE` exists.

**Golden part** `564082-Assembly-3 parts:2` — origin **(−102.03, 593.40, 321.58)**, rotation

| | col x | col y | col z |
|---|---|---|---|
| row X | 0.0000 | −1.0000 | 0.0000 |
| row Y | 0.1021 | 0.0000 | 0.9948 |
| row Z | −0.9948 | 0.0000 | 0.1021 |

i.e. the part's local Y axis lies along file −X (the rotation axis), and the part is turned 84.14° about file X
(local Z is 5.86° from file +Y). The pipe nipple and the solid are coaxial with the file X axis on the line
Y 593.40, Z 321.58 (outer Ø 26.67 mm, bore Ø 20.93 mm); J1 (nipple ↔ solid) lies at X = −210.13.

**Rotary shaft axis.** From the largest cylindrical face of `Rotary Shaft:1` (r 6.000 mm, 10 989 mm²):
direction **±X of the file** (−1, 0, 0 as found; sign not meaningful), through **(Y 448.40, Z 321.58)**, horizontal.
Motor and coupling are on the same line.

**Part rotation axis (positioner axis for the part) — Assumption.** The drive shaft does not carry the part: two timing
belts transfer the motion to a second, parallel axis **±X through (Y 593.40, Z 321.58)**, 145.0 mm above the shaft.
This line is the axis of pulleys :1 / :4, of the holders Part1 and Part2, and of the pipe nipple and solid of the
golden part. It is therefore the axis about which the part turns (J1 coaxial, Mode C), and the candidate for
`positioner_axis`. To be confirmed by the mechanical team (`REF_POSITIONER`, OI-23 / OI-27).

## 4. Notes for the role map (A2)

1. Up axis of the file: +Y (the Pismo plate is 25.4 mm thick in Y, the levelling feet are lowest in Y, the UR stands along +Y).
2. Duplicate `ASSY, Welding tool:1`: once under the UR, once at the top level, identical placement (OI-23).
3. The UR5e assembly contains the head; a role map that maps `UR5E/UR7E:1` as a whole would also take the head.
   The UR links are replaced by `ur_description` (ADR-16); only the base origin is used.
4. Instance names end in `:<n>`; the golden part is `:2` although it occurs once.
5. All children of `564082-Assembly-3 parts:2`, `ASSY, Welding tool:1`, `GSACL25-8-12:1` and `2500001:1` can be
   mapped through their parent (one entry per sub-assembly) — 269 components in total, 156 of them frame fasteners.
