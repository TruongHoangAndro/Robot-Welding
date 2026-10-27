# Change proposal for APLSMN601-DES-001 Rev 0.4 → Rev 0.5

Status: proposal by the software agent, for review by Truong Hoang / Linh Tran. Date: 2026-10-27.
Base documents: DES-001 Rev 0.4 (software architecture), DES-003 Rev 0.4 (function register),
TST-002 Rev 0.2 (collision-avoidance feasibility spike). `docs/reference/` content is not edited.

The proposals CP-1…CP-8 of 2026-10-19 were written against Rev 0.2. Their status after Rev 0.4:

| Old CP | Subject | Status after Rev 0.4 |
|---|---|---|
| CP-1 | ROS 2 Jazzy in Docker | **Closed** — ADR-10 changed to Jazzy in Rev 0.3 |
| CP-2 | OCCT in a separate process (`awm_geometry`) | **Mostly adopted** (ADR-15). Open: pythonocc version → CP-12 |
| CP-3 | Job package cache files | **Partly adopted** (D19, D20 in §5.2). Open: `cache/model.brep`, `cache/collision.stl` names → CP-13 |
| CP-4 | Small corrections | Replaced by CP-14 |
| CP-5 | Partial seams (`range_m`) | **Adopted** as A22 "split into segments"; field must move to mm (`range_mm`) |
| CP-6 | D07 contains the checked joint path | **Adopted in DES-003** (D07 = "tool poses / joint path"); DES-001 §5.1 still says poses only → CP-15 |
| CP-7 | KDL + pick_ik groups | **Withdrawn** — replaced by ADR-12 (analytic IK, ψ × 8 branches, ladder graph) |
| CP-8 | Dense collision check of the whole program | **Still proposed** → CP-16 |

## CP-9 Z-axis convention (decided by Truong, 2026-10-27)

**Decision:** Z is positive upwards.
- `world` and `base`: Z up (REP 103, UR convention).
- Seam frame (A41): X = tangent (travel direction), **Z = bisector of the face normals n1, n2, pointing out of the
  material (up, from the joint towards the head)**, Y = Z × X.
- Beam / tool axis: from the head into the joint = **−Z of the seam frame**. In `tcp_laser`, the tool Z axis is the beam
  direction (UR / ROS tool convention), so `tcp_laser.z = −seam.z` at the nominal angles.

**Conflict resolved:** TST-002 §4 D1 says "Z = −bisector"; DES-001 §4.3 and DES-003 A41 say "Z = bisector (out of the
joint)". Both are consistent once TST-002 is read as the beam axis.

**Text change:** DES-001 §4.3 step 1, DES-003 A41, TST-002 §4 D1: add "Z points out of the material (up); the beam
axis is −Z".

## CP-10 TCP definition and nozzle contact (OI-13, wiki L-001; left to the software team)

**Facts:** the LightWELD emits only when the nozzle / wire guide touches the grounded workpiece (part-contact circuit;
source: wiki L-001 quoting the LightWELD slides 6–7, not yet re-read here; DES-001 §6.2 lists a "workpiece-clamp
contact" interlock). TST-002 §3.1 places the TCP 10 mm beyond the
nozzle tip; TST-002 C2 asks for ≥ 5 mm nozzle clearance; DES-001 §5.4 lets only the TCP link touch the seam faces.
These cannot all hold: with the TCP 10 mm beyond the tip the nozzle never touches the part and the laser cannot fire.

**Proposal (Assumption until the IPG automation documentation, OI-2, confirms it):**

| Item | Proposal |
|---|---|
| TCP `tcp_laser` | On the beam axis **at the nozzle / wire tip**, i.e. the contact point on the seam |
| Focus offset | D06 field `focus_offset_mm` (default 0) moves the TCP along the beam axis; positive = away from the part |
| Collision links | Nozzle split into `nozzle_tip` (last 5 mm) and `nozzle_body` (rest) |
| ACM | `nozzle_tip` ↔ part: contact allowed (no padding). `nozzle_body`, head body, mount, conduit ↔ part / fixture / clamps: not allowed, 3 mm padding |
| Clearance targets | TST-002 C2 "≥ 5 mm" applies to head body, mount and conduit only; `nozzle_body` must stay free of contact with 3 mm padding |
| Contact force | Phase 1 plans the tip exactly on the surface (0 mm pre-load). Whether a compliant (spring) nozzle or force control is needed is a new open item (OI-22) |
| Approach / retract | Unchanged, 30–50 mm along the beam axis; the last approach millimetres and the first retract millimetres are the only motions with tip contact |

**Text change:** DES-001 §5.3 (`tcp_laser`), §5.4 (ACM row), OI-13 → closed by this decision; new OI-22; TST-002 §3.1
TCP row and C2.

## CP-11 Golden part file is a single fused solid

`docs/3D/564082_Solid.step` (AP214, mm) is **one solid**: a bent tube Ø 26.7 / Ø 21.0 mm, about 150 mm long
(straight – bend – straight – bend – straight), 12 faces, 30 edges. All edges between the straight and bent sections
are tangent-continuous (no groove, no fillet). TST-002 §2 describes "564082 Elbow assembly (3 parts)".

Consequences: A11 "split into solids" has nothing to split; the butt joints between the parts can still be picked as
the circular edges at the straight / bend transitions (each split into a 1/4 + 3/4 arc), but the two face normals are
equal there, so the seam frame Z is the surface normal. **Request:** the 3-body assembly STEP (wiki lists "564082 ×3"
in `StepFiles`) so that the real joints exist as separate bodies.

**Update 2026-10-27:** the 3-body assembly is inside the system STEP `docs/3D/APLSMN601_Automated Weldment System.step`
(component `564082-Assembly-3 parts`): `296783_PIPE NIPPLE` (4 287 mm³), `564082-Solid` (36 204 mm³) and
`564082-Elbow` (12 843 mm³). `564082_Solid.step` is `564082-Solid` + `564082-Elbow` fused (49 046 mm³ = sum). The
assembly has two closed butt joints, both Ø 26.67 mm outer (r 13.335): J1 nipple ↔ solid, J2 solid ↔ elbow. The
golden part for Sim-1 is this assembly, not the fused file (decided by Truong, 2026-10-27).

## CP-12 pythonocc-core version (wiki L-004)

ADR-15 and TST-002 E1 name pythonocc-core 7.9 / Python 3.12; the working environment is 8.0.1 / Python 3.10 (conda
`awm_occ`), tested on the generated parts and on 564082. Proposal: ADR-15 → "pythonocc-core ≥ 7.9 in its own conda
environment; tested with 8.0.1".

## CP-13 Job package cache names (§5.2)

Add `cache/model.brep` (D02 cache), `cache/collision.stl` (collision mesh, A13) next to `cache/view/` (D20).

## CP-14 Inconsistencies inside the Rev 0.4 document set

| # | Where | Issue | Proposal |
|---|---|---|---|
| 1 | DES-001 §4.3 para 2 | "reuses the MoveIt configuration and pick_ik setup of the Tac demo" — contradicts ADR-12 (analytic IK, 8 branches) | "reuses the MoveIt configuration; IK enumeration with analytic UR IK (ADR-12)" |
| 2 | DES-001 / DES-003 / TST-002 | Three clearance values: padding 3 mm (§5.4), head ≥ 5 mm (C2), air move ≥ 10 mm (C4) | Keep all three, each with its purpose, in D16 (`padding_mm`, `weld_clearance_mm`, `air_clearance_mm`) |
| 3 | DES-003 A48 vs TST-002 C3 | Wrist-3 ±270° soft limit vs ±180° of start (wiki L-007) | ±270° absolute soft limit (A48, D16) and ±180° travel within one seam (C3) — two different checks |
| 4 | DES-003 Data Items D09 / D10 | Format "URScript / ROS2 trajectory"; DES-001: JSON (`program.schema.json`) | "JSON (ROS2 joint-trajectory segments + events); URScript export = fallback" |
| 5 | DES-001 §4.2 `awm_bringup` vs CLAUDE.md | `sim` = URSim vs `sim` = mock hardware (wiki L-005) | Launch names `sim_mock` (Sim-1) and `sim_ursim` (Sim-2) |
| 6 | DES-001 §1.3 | Reference revisions outdated (wiki L-002) | Update in Rev 0.5 |
| 7 | DES-003 sheet "A0 Overview Phase 1" | Team pipeline A1–A4 with STL input and pose "X, Y, Z, Q1, Q2, Q3" (wiki L-012) | Rename P1–P4, input STEP, pose convention stated (mm + rotation vector or quaternion) |
| 8 | DES-003 A74 "Not in sim" | The chain releases with scope *simulation* (report only) for Sim-1 | Add "simulation release (not valid for production)" to A74 Sim note |
| 9 | Wiki L-003 | Says DES-003 Rev 0.3 history has "units = m"; the history says "units = mm". "units in m" appears only in DES-001 Appendix A (Tracker row) | Correct L-003 in the wiki |

## CP-15 D07 content (§5.1)

DES-001 §5.1 D07: "Weld path (part frame)" → "Tool poses (part frame, mm) **and the joint path chosen by A44**,
tied to the calibration hash". M6 reuses this joint path after an FK check instead of re-solving IK (IK enumeration
plus graph search must not be repeated with a possibly different result).

## CP-16 Verification resolution (former CP-8)

MoveIt validates trajectory waypoints only. M4 / M6 / A71 check every segment densely (≤ 0.02 rad joint step, plus
the 1–3 mm seam pitch) before release. §12 Correctness: "dense collision check of the complete program".

## CP-17 Sim-1 end point (decided by Truong, 2026-10-27)

Sim-1 ends at A71: program D09 + simulation report D15. No production release D10 is produced in Sim-1
(A72–A74 "Not in sim" stays).

**Sim-2 input (decided by Truong, 2026-10-27): simulation-scope release (CP-14 #8).** After A71 passes, M7 writes
D10 with `scope: simulation`. The runtime chain (M8 / M9) in simulation mode accepts only `scope: simulation`; with
real hardware it accepts only `scope: production`, which requires A72 / A73 records and Admin approval. Sim-2
therefore exercises the same release and hash checks as production. Text change: DES-003 A74 Sim stage "Not in sim"
→ "Sim-1: simulation-scope release only"; DES-001 §5.1 D10 adds the `scope` field.

## CP-18 Calibration before path planning (decided by Truong, 2026-10-27)

**Decision:** A4 plans with the active calibration D08. In the simulation stages D08 is the nominal calibration taken
from the cell CAD (CP-23); on hardware, A51 / A52 are done before A4.

**Consequence:** D07 records the D08 hash (CP-15). When the active D08 changes, D07 and everything after it are stale
and must be planned again. Proposal (not decided): allow a fast "re-anchor" that keeps the ψ / IK-branch sequence of
the old D07, re-solves IK with the new D08 and re-runs every check (reach, collision, Δq, wrist-3); any failure →
full re-plan.

**Text change:** DES-001 §4.3 (inputs of M4 include D08), §5.1 D07, §5.2 rule "any change … calibration".

## CP-19 Tack welds are made by hand (decided by Truong, 2026-10-27)

**Decision:** the operator tack-welds the parts by hand after they are clamped on the fixture. The robot does not
tack in Phase 1.

**Consequences:**
- DES-003 A22 / DES-001 §4.2 M2 "sequence incl. tacks" → "sequence" (no robot tacks in Phase 1).
- A81 operator sequence: load parts → clamp → tack by hand → Start request.
- Tack beads are not in the CAD. With nozzle contact (CP-10) the nozzle tip rides over them: add to OI-22
  (compliant nozzle / contact force). Tacking can move the part: input to A53 / A54.
- Where and with which tool tacks are made (inside the Class 4 enclosure?) is a process and safety question for
  REQ-002, not for the software.

## CP-20 Rotary positioner in Phase 1 (decided by Truong, 2026-10-27; closes OI-20)

**Decision:** the 1-axis rotary positioner of the mechanical design is part of Phase 1. This replaces Truong's earlier
conditional position of the same day. The approval of the Engineering Manager is to be recorded in DES-001 Rev 0.5.

**Hardware as modelled (system STEP, Fusion):** motor `60CM30X`, coupling `GSACL25-8-12`, `Rotary Shaft`, two
timing-belt stages (`HTPK20S5M150` pulleys, `Timing Belts`), holders `Part1` / `Part2` / `Part3` on the `Jig`.
Rotation axis horizontal (CAD X). On the golden part, J1 (nipple ↔ solid) is coaxial with the axis; J2 (solid ↔ elbow)
is 76.6 mm off the axis with its own axis perpendicular to it.

**Proposed Phase 1 motion modes (no synchronised 7-axis interpolation, §6.3):**

| Mode | Use | Motion during emission | Sequence |
|---|---|---|---|
| C — positioner weld | Seams coaxial with the axis (J1) | Robot at rest; positioner turns at ω = travel speed / seam radius, plus closing overlap | Positioner at rest → WeldStart → emission on (PLC) → M9 runs the positioner weld segment → positioner at rest → WeldEnd → emission off |
| B — rotate, stop, weld | Other seams (J2) | Positioner at rest at a planned angle; robot welds the arc segment | As today (§6.2) |
| A1 / A2 — continuous 7-axis | — | — | Phase 2 |

**Ownership (decided by Truong, 2026-10-27): ROS 2 controls the positioner motor.** This is the ADR-11 design moved
into Phase 1:
- `awm_positioner`: ros2_control hardware interface for the joint `positioner_joint`, loaded in the **same
  controller manager** as the UR hardware; Phase 1 uses its own `positioner_trajectory_controller`
  (joint_trajectory_controller). Sim-1 / Sim-2: `mock_components` for this joint (ADR-13).
- M9 executes positioner segments exactly like robot segments (it is motion, not a process event). M10 / PLC keep
  only the LightWELD.
- Recommended drive (OI-14): closed-loop drive with encoder on EtherCAT (CiA 402) through `ethercat_driver_ros2`, on
  its own IPC network port, PREEMPT_RT kernel (already planned, §8). It keeps the path open to continuous 7-axis
  motion (Option A1, Phase 2) without changing the software structure.
- Safety (hardware, independent of the IPC): drive STO wired into the safety chain (E-stop, door); drive ready / fault
  hard-wired to the PLC so the PLC switches emission off on a positioner fault; the PLC heartbeat watchdog (§6.1)
  also covers a ROS crash during a Mode C weld.
- Rule 6 holds: WeldStart / WeldEnd only while robot **and** positioner are at rest. Speed-scaling rule extended: a
  Mode C weld segment starts only at 100 % positioner speed (no override on weld segments), any deviation → WeldEnd +
  FAULT.

**Software impact:**
- D14: revolute joint `positioner_joint`, rotating links (shaft side, holders), frames `positioner_axis` /
  `positioner_flange`; fixture and part are children of `positioner_flange`. Sim-1: mock joint. Sim-2: URSim has no
  positioner → `mock_components` joint in the same controller manager as `ur_robot_driver`.
- M4: A49 moves to Phase 1. Mode C: one robot pose per seam, collision / clearance of head vs. the rotating part and
  holders checked over the full angle range. Mode B: positioner angle per segment from a discrete set, searched
  together with ψ × IK branches. A47: Mode C speed = WPS travel speed / seam radius.
- D07 / D09: points and segments carry the positioner angle (deg); D09 gets positioner segments and the Mode C weld
  event.
- M5: A55 (positioner axis calibration) moves to Phase 1; in simulation nominal from CAD.
- TST-002 C8: report coverage robot-only and with the positioner.
- Open (OI-14, narrowed): motor driver and its fieldbus (EtherCAT proposed), encoder or open loop, belt backlash. Accuracy check:
  angular error δ gives r·δ at the seam; at r = 13.335 mm, 0.1 mm ⇔ 0.43°.

**Text change:** DES-001 §1.2, §4.2 (`awm_positioner` Phase 1), §6.2 (Mode C sequence), §6.3, ADR-11, OI-14, OI-20
(closed); DES-003 A49, A55 → Phase 1 with Sim stage "Sim-1 MoveIt mock".

## CP-21 Seam picking by Engineer only (decided by Truong, 2026-10-27)

**Decision:** only the Engineer role picks and edits seams. **Text change:** DES-001 §4.5 S2 "Roles" → "Engineer";
§7.3 Operator row: delete "seam picking (S2) only with the 'Seam setup' permission"; §7.4 `/jobs/{id}/seams` PUT →
Engineer.

## CP-22 Part placement flow (A15) (decided by Truong, 2026-10-27)

**Problem:** A15 sits in A1, but a reach check needs the seams (A2) and the planner (A4); and the software placement
must match the physical fixture, so it cannot be moved freely once the fixture exists. Seams (D04) and process data
(D06) are in the `part` frame, so a placement change makes only D19 → D07 → D09 stale, never the seam picking.

**Decision:** combined flow for a new part number:

1. A1 import STEP (no placement needed for seam picking).
2. A2 the Engineer picks seams.
3. **Placement study**: candidate placements are evaluated with the planner. Sim-1: candidates entered by hand by the
   Engineer, full A4 run per candidate (PA1). Later (R3 Smart Path): automatic search over orientations with a coarse
   planner setting and a ranked reach table (PA2).
4. Mechanical design builds the fixture for the chosen placement (Fusion).
5. **Import placement from the fixture assembly** (CP-23): D19 is fixed by the fixture CAD (PA3). A placement change
   after this step requires a new fixture revision.
6. A3, then A4 full planning, A6, A7.

**Data:** D19 gets `source` = `study` | `fixture_cad`, plus fixture assembly file name, revision and SHA-256 when
`fixture_cad`. A simulation-scope release (CP-17) accepts both; a production release requires `fixture_cad`.

**Text change:** DES-003 A15 split into "Placement study" (after A2) and "Import placement from fixture CAD" (new
function ID to be assigned in DES-003 Rev 0.5); DES-001 §4.2 M1 / M4, §4.3, §5.1 D19.

## CP-23 Cell and fixture geometry from Fusion STEP assemblies (decided by Truong, 2026-10-27)

**Decision:** nominal cell geometry comes from Fusion (the CAD tool of the mechanical team, confirmed 2026-10-27), exported as STEP assemblies and read by `awm_geometry`
(pythonocc XCAF: component tree, names, placements). No CAD → URDF plug-in.

| Assembly | Content | Produces |
|---|---|---|
| Cell assembly (once per cell revision) | Pismo frame, table, robot mounting position, enclosure parts within reach | D14 static links + collision meshes, `world_T_base` |
| Fixture assembly (one per part number / fixture revision) | Fixture, clamps, the job part | D19 placement + collision scene |
| Head assembly | LightWELD head, mount, nozzle, conduit section (when IPG CAD arrives, OI-15) | Head collision links, nominal `tool0_T_tcp` |

**Rules:**
- The UR5e robot itself is **not** taken from CAD: kinematics, limits and collision meshes come from `ur_description`.
  The CAD gives only the base mounting pose.
- Export: STEP AP214 or AP242, assembly structure kept, units mm.
- Frames are given by **marker components**: a small part with a fixed name whose part origin is the frame
  (component placements survive STEP export; Fusion construction geometry and joint origins are not exported — to be verified).
  Names: `REF_WORLD`, `REF_UR_BASE` (aligned with the UR controller `base` frame, which is `base_link` rotated 180°
  about Z in `ur_description`), `REF_FIXTURE`, `REF_PART` (origin of the job STEP), `REF_TOOL0`, `REF_TCP`.
- Import checks: every required marker present exactly once; the part in the fixture assembly matches the job STEP
  (fingerprint / volume / bounding box); units mm; otherwise the import is refused with a clear message.
- The nominal calibration D08 of the simulation stages (CP-18) is computed from the markers (`base_T_fixture`,
  `tool0_T_tcp`). On hardware, A51 / A52 measurements replace it.

**Next step:** mechanical design exports one simple test assembly (table + fixture box + part + markers) to check
that marker placements and names survive the Fusion STEP export.

**Update 2026-10-27:** the first system STEP (`docs/3D/APLSMN601_Automated Weldment System.step`, 98 MB, AP214, mm)
was exported by Autodesk Translation Framework (Fusion). Read with pythonocc XCAF in
56 s: 269 component nodes, product names and placements kept (instance names get a `:n` suffix). Findings:
- **Y is up** in this file (casters at the lowest Y, robot above the frame). The import must apply a fixed
  CAD → `world` rotation (CAD Y → world Z); the CAD tool / export axis must be stated per file.
- No marker components yet. The UR base pose can be read from the UR CAD base body (`C-1000257`), but the UR CAD
  axis convention vs the controller `base` frame must be confirmed (marker `REF_UR_BASE` still needed).
- `ASSY, Welding tool` appears twice at the same pose (under `UR5E/UR7E` and at top level): duplicate geometry.
- The LightWELD head body is a surface model (212 shells, BRepCheck invalid): usable for collision only after
  tessellation / convex hulls, as planned in §5.4.
- The upper laser enclosure is not in the model (only the lower frame `2600001` and top plate).

**Text change:** DES-001 §5.3 (frames: source of nominal values), §5.4 (data sources table), §4.2 M1 / `awm_description`.
