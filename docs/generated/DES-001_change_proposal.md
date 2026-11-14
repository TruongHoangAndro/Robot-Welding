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

---

# Change proposal DES-001 Rev 0.5 → Rev 0.6 (architecture review, 2026-10-30)

Status: proposal by the software agent, for review by Truong Hoang / Linh Tran. Date: 2026-10-30.
Base: DES-001 Rev 0.5 (`docs/generated/release_rev0.5/`, working baseline, unchanged).
Draft: `docs/generated/drafts/APLSMN601-DES-001_Software_Architecture_Rev0.6_DRAFT.md`, built by
`docs/generated/drafts/make_des001_rev06.py` from the Rev 0.5 release (checked text replacements).
DES-003 is not changed in this step; its impact is listed per CP and goes into DES-003 Rev 0.6 after the decisions.

Review summary: the architecture principles are sound (offline / online split, PLC owns the laser, data-driven job
package with hashes, simulate first, single mm ↔ m adapter, release scopes). Four high risks: (1) the core planner
(ADR-12) is still unproven while other work builds on it; (2) absolute accuracy of offline programming from CAD is
not addressed; (3) the software view of safety is thin, mainly for Mode C (positioner turns during emission);
(4) scope versus schedule and the lack of an independent human review.

| CP | Subject | Kind | Needs decision | DES-001 Rev 0.6 | DES-003 impact |
|---|---|---|---|---|---|
| CP-24 | Accuracy chain: calibrated UR kinematics, part-frame correction hook | Design | Yes (Truong) | ADR-18, AD5, M5, 5.3, R14 | A51/A53/A54 text, D08 content |
| CP-25 | First error budget | Analysis | Values: welding engineer (OI-30) | 12.2 | A54 |
| CP-26 | Safety concept, software view | Design | PLr: safety engineer (OI-31) | 12.1, 1.4 | — |
| CP-27 | Watchdog also stops the positioner; Mode C burn-through check in PLC | Design | Yes (PLC / safety) | 6.1 | A82, A84 |
| CP-28 | Planning as asynchronous, reproducible run | Design | Yes (Truong) | ADR-19, 4.3, D07 | D07 content |
| CP-29 | Schema migration, atomic writes, job lock, release manifest | Design | Yes (Truong) | ADR-20, 5.2, D10 | D10 content |
| CP-30 | `awm_api` split into gateway + ROS bridge | Design | Yes (Truong) | ADR-21, 4.2 | — |
| CP-31 | Test strategy, observability, measurable quality targets | Design | Review | 12, 12.3, 12.4, D09, D12 | D09, D12 content |
| CP-32 | Web API rules: `/api/v1`, OpenAPI contract, error model, `202` + run id | Design | Review | 7.4 | — |
| CP-33 | Phase 1 positioner interface sized for Modes B / C only | Design | Yes (Truong) | ADR-22 | — |
| CP-34 | Sim-1 thin slice | Scope | Yes (Truong / PM) | Principle 9, OI-35 | "Sim stage" column |
| CP-35 | Human review gate for safety-relevant code | Governance | Yes (Engineering Manager) | Principle 10, ADR-23, OI-32 | — |

## CP-24 Accuracy chain (ADR-18)

**Finding:** AD5 asks for ±0.1–0.2 mm. The UR5e repeatability is ±0.03 mm (Fact, data sheet), but offline
programming from CAD depends on **absolute** accuracy. With the nominal `ur_description` kinematics this error is not
known and is expected to be larger than the band (Assumption, to be measured). Rev 0.5 does not mention the
robot-specific calibrated kinematics that UR stores in each controller. Part tolerance and tack distortion (CP-19) add
error, while seam tracking (A94) is Phase 2 and touch sensing is R4.

**Proposal:** on hardware, extract the calibrated kinematics with `ur_calibration` and use them in D14 / D08 (hash in
D08); simulation stays `NOMINAL`. `fixture_T_part` = nominal · correction; the correction comes from A53 (identity for
the fixed jig, measured for touch-off or a sensor) without changes in M4 / M6. Whether touch-off moves from R4 to R2 is
decided by the error budget (CP-25).

## CP-25 Error budget (12.2)

**Proposal:** a table of sources E1…E9 with Fact / Assumption / [TBD] marks and a decision rule, reviewed at every
calibration (A54). Only E1 is a Fact today. Values to be filled by OI-30 together with OI-16 (allowed seam deviation).

## CP-26 Safety concept, software view (12.1)

**Finding:** REQ-002 is still planned; DES-001 has no allocation of safety functions. Mode C lets the positioner turn
during emission under ROS 2 control.
**Proposal:** state that the IPC software is not a safety-related part; allocate SF1…SF6 to hardware / PLC / UR safety
configuration; PLr from REQ-002 (OI-31).

## CP-27 Watchdog and Mode C burn-through check (6.1)

**Proposal:** on IPC heartbeat loss the PLC switches emission off **and** removes the positioner drive enable. During a
Mode C weld segment the PLC switches emission off when the drive does not report motion in time, and raises a fault
when the positioner moves outside a weld or index segment.

## CP-28 Planning runs (ADR-19)

**Finding:** planning is long, started from a Web UI by several users, on one `move_group`; Rev 0.5 does not define
queueing, cancellation, job locking or reproducibility.
**Proposal:** `run_id`, one queue per `move_group`, job lock while a run is active, cancel, recorded seed, software
version and D16 / scene hashes in D07. Same inputs → same D07 (regression test).

## CP-29 Data integrity (ADR-20, 5.2)

**Proposal:** `schema_version` = `MAJOR.MINOR` with migration scripts in `awm_core` for MAJOR changes; atomic writes
(temp file + rename) and one writer per job; `release.json` holds a manifest with the SHA-256 of every released file,
checked by the runtime; archive copy on the shared drive. Git-backed job repository kept as a Phase 2 option.

## CP-30 `awm_api` split (ADR-21)

**Proposal:** FastAPI gateway process (HTTP / WebSocket, roles) and a separate rclpy bridge node; the bridge is the only
ROS client of the UI. Avoids mixing the asyncio loop and rclpy executors in one process.

## CP-31 Test strategy, observability, quality targets (12, 12.3, 12.4)

**Proposal:** test levels unit → contract → planner regression (golden part, reproducibility) → `sim_mock` integration
→ URSim → HIL with the PLC → FAT; negative test for every rejection rule. Structured logs with `job_id` / `run_id` /
`seam_id`, `/diagnostics`, software version stamps in D07, D09, D12, D18, daily backup with a restore test. New quality
rows: reproducibility, performance (≤ 30 s per seam, ≤ 5 min per job — Assumption), integrity, observability.

## CP-32 Web API rules (7.4)

**Proposal:** `/api/v1` prefix; generated OpenAPI is the contract and is reviewed like `awm_interfaces`; one error model
(RFC 9457 problem details); long operations return `202` + run id; new endpoints `/jobs/{id}/runs` (GET / DELETE).

## CP-33 Phase 1 positioner interface (ADR-22)

**Finding:** ADR-11 picks EtherCAT + PREEMPT_RT so that Option A1 (continuous 7-axis motion) stays open for Phase 2.
Phase 1 Modes B / C need no tight robot ↔ positioner synchronisation. OI-14 (drive, fieldbus) is still open.
**Proposal:** keep ROS 2 control (`awm_positioner`, ros2_control) but choose the Phase 1 drive interface on Phase 1
needs; the drive may run its own position / velocity profile. EtherCAT + A1 remains the Phase 2 target. This is a
detail of ADR-11, not a reversal; needs Truong's decision.

## CP-34 Sim-1 thin slice (principle 9)

**Finding:** about 5 weeks to the Phase 1 target, 13 modules, 10 screens, positioner, cell CAD import, release flow.
**Proposal:** Sim-1 = golden part 564082, J1 Mode C, J2 robot-only, screens S2 + S4 with a minimal job list; S3,
S5–S10 and automatic A18 after Sim-1 unless needed. Align PM-004 (OI-35).

## CP-35 Human review gate (ADR-23)

**Finding:** all functions are owned by the software agent (OI-28) and "Reviewed by" is [TBD]. The cell has a Class 4
laser and moving axes.
**Proposal:** CODEOWNERS and a protected branch; a named engineer approves every merge in M8, M9, M10,
`awm_positioner`, `awm_interfaces`, the PLC program and D16 limits (OI-32). Function ownership (OI-28) is unchanged.

## Document corrections in Rev 0.6

- Header: "Approved by" marked pending; status = draft.
- ADR-12: past decision date (29 Oct) replaced by OI-29 (new TST-002 decision date).
- New OI-29…OI-35 (TST-002 date, error budget values, REQ-002 PLr, reviewers, launch-name alignment `sim` vs
  `sim_mock`, code migration Rev 0.2 → Rev 0.5/0.6, Sim-1 scope).
- Not done in this step (proposed for later): move decision log and status text out of DES-001 into this change
  proposal, so DES-001 describes only the architecture; redraw of the figures (OI-17); release names R2 / R4 (charter CR).

# Architecture Review Board 2026-10-30 (AI reviewers) -> DES-001 Rev 0.6

Status: board consensus applied to the Rev 0.6 draft by the software agent (technical editor), 2026-11-02. **The
board members were AI reviewer agents (roles, not people), run by the software agent. Their result is input for the
human review by Truong Hoang and Linh Tran; nothing here is decided until they accept it.** Record of the board:
`docs/generated/arch_review_board_2026-10-30.md`.

**Method.** Three AI reviewer roles (CTO / Chief Software Architect; VP Quality, Functional Safety & Manufacturing
Engineering; Program / Delivery Director) reviewed the Rev 0.6 draft and the code independently (round 1, 41
proposals), cross-examined each other (round 2, 15 new proposals, concessions and conflicts), and a chair merged the
result into 25 decisions (B-01…B-25), a CP review (CP-24…CP-35) and 8 escalations (E-01…E-08). The editor applied the
decisions to `make_des001_rev06.py`; the draft is rebuilt from the Rev 0.5 release. Board verdict: **approve with
conditions, 5 / 10** (all three members 5 / 10 after round 2). New CP numbers start at CP-36.

## Board decisions

| B-id | Title | Decision | Priority | CP |
|---|---|---|---|---|
| B-01 | TST-002 decision on Fri 2026-11-06 with frozen inputs | accept_modified | P0 | CP-36 (new) |
| B-02 | Joint-aware seam frames for welds between separate solids | accept_modified | P0 | CP-37 (new) |
| B-03 | Planner state space and acceptance metrics (unwrapped joints, labels, pitch rule, unpadded clearance, singularity metrics, tie-break, displacement-bound step) | accept_modified | P0 | CP-38 (new) |
| B-04 | Nozzle contact checked explicitly; no ACM exemption | accept_modified | P0 | CP-39 (new) |
| B-05 | ψ window per WPS (filler wire, tip type); spike reports both cases | accept_modified | P0 | CP-40 (new) |
| B-06 | M4 core behind KinematicsBackend / SceneBackend; independent M7 re-check; runtime dependency rule | accept_modified | P0 | CP-41 (new) |
| B-07 | One owner per scene object (D14 / D19), shared scene manifest, default-deny CAD mapping | accept_modified | P0 | CP-42 (new) |
| B-08 | Migration order; T0.1 limited to kept code; D16 and D08 split | accept_modified | P0 | CP-43 (new) |
| B-09 | Immutable per-revision release bundle, one active pointer per scope | accept_modified | P0 | CP-29 (modified) |
| B-10 | One planning-context record, one compare function, policy table | accept | P0 | CP-44 (new) |
| B-11 | CLI subprocess job runner; no gateway / bridge split; achievable reproducibility | accept_modified | P0 | CP-28 (modified) |
| B-12 | Ordered Sim-1 slice: J1 Mode C first, J2 robot-only "100 % or rejected" | accept_modified | P0 | CP-34 (modified) |
| B-13 | Staged Web UI; v2 data contracts frozen 2026-11-04 | accept_modified | P0 | CP-32 (modified) |
| B-14 | Delivery stages Sim-1…HW-2; one DES-001 revision per gate; disposition table | accept_modified | P0 | CP-45 (new) |
| B-15 | D09 is the exact commanded trajectory; M9 sends it verbatim | accept | P1 | CP-46 (new) |
| B-16 | Release scope `trial`; production as promotion of the same bundle | escalate (E-02) → **option A, Truong 2026-11-03** | P1 | CP-47 (accepted 2026-11-03) |
| B-17 | Typed `awm_interfaces` v1; emission tied to axis motion; M8 heartbeat; no automatic retry; EmissionLost | accept_modified | P1 | CP-27 (modified) |
| B-18 | Named humans for PLC program, DES-002, safety, hardware execution; weld-event table; ADR-23 dates | escalate (E-03) | P1 | CP-35 (modified, Proposed) |
| B-19 | Minimal Sim-2 runtime: Python M9, M8-minimal, table-driven M10 sim, mixed-mode refusal, fault injection | accept_modified | P1 | CP-48 (new) |
| B-20 | ADR-22 conformance wording; Mode C ramps planned | accept_modified | P1 | CP-33 (modified) |
| B-21 | Accuracy chain: budget by direction and mode; IK refinement at HW-1; no runtime per-part correction | accept_modified | P1 | CP-24 (modified) |
| B-22 | Dated dependency register; customer demonstration agreement | accept | P1 | CP-49 (new) |
| B-23 | Company repository and CI runner; hardened IPC image before HW-1; network security | accept_modified | P1 | CP-50 (new) |
| B-24 | M11 / M12 deferred; Phase 1 weld log = M8 JSON lines | accept_modified | P2 | CP-31 (modified) |
| B-25 | Hardware-stage gate items: calibration change, TCP verification, UR safety configuration, installation block, seam re-match | accept_modified | P2 | CP-51 (new) |

All 25 decisions are written into the Rev 0.6 draft. B-16 and B-18 are escalations; their text is in the draft only
as **Proposed** amendments pending OI-38 / OI-39 — the decided ADR-17 and OI-28 stay in force until then.

## Outcome for CP-24…CP-35

| CP | Outcome | Draft change |
|---|---|---|
| CP-24 | Modified (B-21) | ADR-18 rewritten: calibrated kinematics mandatory on hardware, IK through KinematicsBackend with refinement before HW-1, checksum in the context record, no runtime per-part correction (OI-43); 5.3 paragraphs; M5 row |
| CP-25 | Modified (B-21) | 12.2 split into Robot weld / Mode C tables with Lateral / Along-beam / Allocation; E10–E13; E2 = Fact (bound); values due at the Sim-2 exit (OI-30) |
| CP-26 | Modified (B-18) | 12.1: sentence "SF3–SF5 are process interlocks…"; SF4 / SF5 / SF6 rows updated; full split at HW-1 |
| CP-27 | Modified (B-17) | 6.1 paragraph "Emission follows the welding axis"; 6.2 step 3 without retry; 7.1 weld_event typed; 7.2 new hard-wired rows |
| CP-28 | Modified (B-11) | ADR-19 = CLI subprocess runner, one slot, `409`, SIGTERM cancel; 4.3 "Planning runs"; reproducibility row |
| CP-29 | Modified (B-09) | ADR-20 = frozen per-revision bundle with active pointers; 5.2 rules and tree; D10 storage |
| CP-30 | **Withdrawn** | ADR-21 row kept (ADR-22 and ADR-23 are referenced) and marked **Rejected for Phase 1**; UI backend row = one FastAPI process |
| CP-31 | Modified (B-19, B-24, B-11) | 12.3 staged with fault-injection and HIL conformance rows; 12.4 staged; D15 validity statement; image digest |
| CP-32 | Modified (B-13) | 7.4 Stage column, runs endpoint, no jog, `production` refused until HW-2 gate code |
| CP-33 | Modified (B-20) | ADR-22 conformance wording; OI-14 purchase requirement; decide by 2026-11-20 |
| CP-34 | Modified (B-12, B-13) | Principle 9 ordered slice; 4.3 step 3 Mode B with entered angles; D19 `verified`; Section 11 A17 / A18 / A49 |
| CP-35 | Modified (B-18) | Principle 10 and ADR-23: effective 2026-11-23, wider scope, Sim-1 sign-off, PLC program needs a human author |

## Escalations (not decided — open items in the draft)

| E-id | OI | Question | Options | Board recommendation | Decider |
|---|---|---|---|---|---|
| E-01 | OI-37 | TST-002 decision on 2026-11-06 with frozen inputs, missing inputs = slip ≤ 3 days, not Pivot? | A yes; B decide with whatever exists, missing = Pivot; C no fixed date | A | Linh Tran, with Truong Hoang |
| E-02 | OI-38 | Amend ADR-17 with a `trial` scope; production = promotion of the same bundle? | A amend now; B decide at HW-1 gate; C keep, run A72 / A73 outside M8 / M9 | A (B fallback); C not acceptable | Truong Hoang — **decided 2026-11-03: A** (schema v2: scope enum + `promoted_from`; DES-001 Rev 0.6 draft ADR-17 amended, OI-38 closed) |
| E-03 | OI-39 | Amend OI-28: named human PLC & safety engineer, commissioning engineer, LSO? | A amend (names by 2026-11-20, LSO before HW-2); B PLC program only; C keep | A | Linh Tran, with Truong Hoang |
| E-04 | OI-40 | Phase 1 deliverable on 2026-12-04? | A Sim-2 system, HW stages in PM-004; B move Phase 1 end after HW-2; C compress HW into December | A, with the customer agreement (OI-5) | Tac Huynh and Linh Tran |
| E-05 | OI-41 | Filler wire on J1 / J2, and from which side? | A autogenous, free ψ; B wire leading, ±15°; C unknown: report both, decide on wire case | Answer by 2026-11-04, else C | Truong Hoang (welding engineer, Khang Doan) |
| E-06 | OI-42 | Rebuild IPC / CI image from public `ros:jazzy` before HW-1? | A yes, in parallel, HW-1 entry; B demo image on the IPC; C switch now | A | Truong Hoang |
| E-07 | OI-43 | Runtime per-part IK re-solve as exception to design rule 1? | A no exception; B bounded exception; C offline re-plan per part | Decide at HW-1 gate; default A | Truong Hoang |
| E-08 | OI-44 | Demo part 564082 or 152056; part-detection sensor in Phase 1? | A 564082 only; B add 152056; C sensor in Phase 1 (own CP) | A, plus B if STEP by 2026-11-13 | Tac Huynh |

OI numbering note: B-25 names the requalification item OI-36, so the escalations start at OI-37 (B-14's reference
to "OI-37" for the Phase 1 deliverable is written as OI-40 in the draft).

## DES-003 and PLAN.md impacts (not applied in this step)

DES-003 Rev 0.6 (after the human decisions):
- Descriptions: A11, A21, A24, A41 (joint-aware frames, B-02); A41, A43, A44, A45, A48, A49 (state space, pitch,
  clearance, singularity, Mode C tolerance, B-03); A45, A71 (contact check, B-04); A31, A33, A42, A44, A49 (ψ window,
  B-05); A47, A62, A71, A83 (D09 as commanded trajectory, B-15); A72, A73, A74, A81 (trial scope, if E-02 = A);
  A63 (no automatic retry; table-driven M10 sim), A82, A84 (B-17, B-18); A85, A87, A91, A92, A95 (JSON-lines log,
  record types, B-24); A15, A18 (D14 / D19 boundary, default-deny, B-07); A51–A55 targets "from the error budget",
  A53 typed LOCATE_PART (B-21).
- New function A56 "Verify TCP" (Phase 1, R2, HW-2; B-25).
- Data items D04, D05, D06, D07, D08, D09, D10, D12, D13, D14, D16, D18, D19 descriptions.
- "Sim stage" column: A63 → "Sim-1 markers; Sim-2 M10 sim"; A81, A82, A84 → "Sim-2 (minimal)"; A17, A18, A49 per
  B-12; legend gains HW-1 / HW-2; align with the Stage column of DES-001 Section 11.
- Performers of A51–A55, A72, A73, A81–A84: named commissioning engineer (if E-03 = A).

PLAN.md (rebase on DES-001 Rev 0.5 / 0.6):
- §1: TST-002 row as the Sim-0 exit (2026-11-06); contract freeze 2026-11-04; HW-1 / HW-2 rows (dates TBD).
- §2: keep / replace column per package (closes OI-34).
- T0.1: scope = `awm_core` + kept code; schemas v2, context record, bundle layout, D16 / D08 split; regenerate test
  jobs. T0.2: scripted A18 with default-deny. T0.3: D14 / D19 boundary, "contact-model test" instead of ACM test.
  T0.5: `awm_interfaces` v1 typed. T0.6: URSim + `ur_robot_driver` moved into WP0; CI runner.
- WP1: spike = M4 core behind backends (2–5 Nov); pitch rule; pin `ur-analytic-ik`. T1.5 before T1.6; T1.6 split into
  entered angles (conditional) and automatic (R3).
- WP2: T2.1 joint-aware topology and scene manifest; T2.4 uses the CAD placement (`verified: false`); T2.6 writes the
  bundle layout.
- WP3: S1-min / S2 / S4-lite, from 2 Nov on the v2 example files; CLI job runner; units module and FK test.
- WP4: Python M9, M8-minimal, table-driven M10 sim, mixed-mode refusal, JSON-lines log, fault-injection script
  without retry; runbooks and support-bundle CLI.
- New §6 "Risks and triggers"; PROGRESS.md weekly dependency check.
- TST-002 Rev 0.3 (new revision copied into `docs/reference/` only from the Drive master): frozen inputs §2 / §4 / §5,
  entry checks, two ψ cases, two head models, outcome table §6, records per result.

# Changes found during the spike (from 2026-11-02)

## CP-52 Joint detection between solids without a general fuse (modifies B-02 / CP-37)

**Status:** accepted by Truong Hoang on 2026-11-02 (implementation evidence: WP-S S.1, TST-002 Rev 0.3 G1 PASS).
**Finding:** B-02 asked M1 to build a joint-aware weldment shape by a general fuse of the part solids (fuzzy value
0.05 mm), with every face keeping its source-solid id. S.1 shows that the fuse is not needed: the joint edges are found
directly between the separate solids (an edge of solid A coinciding with an edge of solid B within 0.05 mm), and at
each seam point n1 / n2 are the normals of the exterior faces of the two solids, i.e. the face next to the edge that
does not touch the other solid (`awm_geometry.joints`, solids read with `awm_geometry.assembly`).
**Result:** on the golden part 564082 (3 solids, J1 and J2) the seam-frame Z is radial within 0.000° at 12 points per
joint (per-solid rule of Rev 0.2: 45°). No boolean operation, no fuzzy-value tuning, every normal keeps its source
solid by construction.
**Proposal:** DES-001 Rev 0.6 M1 row: "joint detection between solids (general fuse; each face keeps its source-solid
id, B-02)" → "joint detection between solids (coincident edges within 0.05 mm, n1 / n2 from the exterior faces of
each solid; no general fuse; B-02 as modified by CP-52)". 4.3 step 1 is unchanged. PLAN.md S.1 text and the board
record line "M1 adds a joint-aware fused weldment shape" follow. The 0.05 mm coincidence tolerance is an
**Assumption** (tolerance of the Fusion export), to be revisited if a customer STEP has gaps or overlaps at a joint;
the tube-rule fallback (n1 = n2 = radial) stays documented in TST-002 Rev 0.3 §3.2.
**DES-003 impact:** A11 / A21 / A41 descriptions use "joint detection" instead of "fused weldment shape".

## CP-53 Closed seams welded robot-only are split into arcs; pitch refined to the joint step (TST-002 F1, C3, C8)

**Status:** accepted by Truong Hoang on 2026-11-03 (J2 = 2 arcs; the number of arcs is entered by the Engineer in the
UI, not chosen by software — Truong, 2026-11-03); pitch rule decided by the lead within F1 (≤ ceiling).
**Finding (WP-S S.3):** a closed 360° seam welded by the robot alone makes the tool orientation turn once about the
seam axis. That loop is not contractible in SO(3) (generator of π1(SO(3)) = Z2), while the forward-kinematics image of
a joint path that returns to the same unwrapped state is contractible. Hence **at fixed ψ** some joint must make a net
360° turn (topological fact); F11 (wrist 3 ±180° within one seam, fiber / conduit) and F3 (closure on the same
first-layer state) reject every such path. With ψ allowed to vary, a net ±360° ψ unwinding would make the loop
contractible; with the 10° ψ grid (step > Δq_max 5°) this did not happen on the synthetic r 13.3 mm placement
(all 36 ψ, 288 points: no path, open or closed) — an **empirical** result for that placement, not a proof (review of
S.3). Ψ-WIRE (±15°) cannot unwind 360°. Measured: ψ 90°, 288 points, the DP
stops at layer 145 on wrist-3 travel; without F11 it closes only on q6 + 360°. Second finding: at the F1 pitch
(1.16 mm, r 13.3 mm) joint steps reach 13°; at 0.29 mm they are ≤ 3.3°.
**Decision:** (1) The Engineer decides whether a seam is split: seam attribute `arcs` (integer ≥ 1, default 1) and
`arc_overlap_mm` in D04, entered in the seam editor S2 (A22 split). The software never splits a seam by itself; it plans
every arc (equal lengths from the seam start), accepts the seam only when every arc is 100 %, and otherwise rejects the
failing arc(s) with their failing points so the Engineer can change the number of arcs, the start point or the
direction. Emission stops between arcs (WeldEnd / WeldStart at rest, DES-001 §6.2). For the spike, J2 is entered with
2 arcs (each ≤ 180° + overlap); C2 / C3 apply per arc; C8 "2 segments" gates J2. A closed seam with `arcs` = 1 welded
robot-only is rejected (reason recorded: net 360° joint turn needed, F3 / F11).
Mode C (J1) is unaffected: the robot is at rest. (2) F1 is a ceiling: the planner refines the pitch until consecutive
IK solutions of one label move ≤ Δq_max and records the pitch used.
**Pitch rule refinement (lead, 2026-11-04, no objection from Truong):** the Δq_max check that drives the refinement
counts only pairs of scene-valid nodes (F4, F5, F9 pass); kinematic-only pairs do not force a finer pitch. Planner
parameter `rule = kinematic | scene_valid` (default `kinematic` keeps the S.5 records valid); used as `scene_valid` in
the S.5b Mode B screening (ws-s4c-checks 3b74a34) and recorded per run (`pitch_rule`). Pitch floor 0.05 mm (Assumption).
**Open:** weld quality at the arc restart / overlap (welding engineer; OI-11 / OI-16); whether a 360° wrist-3 turn is
acceptable for the fiber and wire conduit (Khang Doan / Nghia Nguyen) — if yes, a later CP can relax F11.
**F6 wording (S.3 review):** the shoulder-singularity metric is the distance |u| of the wrist centre from the plane of
the joint-1 and joint-2 axes (≥ 50 mm, Assumption); the literal "distance from the base Z axis" is always ≥ d4 =
133.3 mm on the UR5e and could never trigger. TST-002 Rev 0.3 F6 row updated.
**DES-001 impact:** §4.3 step 4 / 5 note "closed seams welded robot-only are split into arcs (CP-53)"; A22 (split) and
A42 (overlap) descriptions; TST-002 Rev 0.3 F1, C3, C8 rows updated in the draft.


## CP-54 S10-lite cell model role editor pulled forward (decided by Truong, 2026-11-03)

**Decision:** the role map of a cell / fixture CAD assembly (`cad_map`, ADR-16 / CP-23) is edited in a UI page
(S10-lite) instead of by hand: after import the component tree is shown in 3D; the user assigns a role to each
component (part, positioner rotating set, cell static, head CAD, ignore; picks the robot base and the positioner axis)
and may give it a display name. The STEP file is never modified; display names live in the map. Default deny.
PLAN Rev 0.6 had S10 after Phase 1 (CLI / API only) — S10-lite moves into WP3 now, before Sim-1, and runs in parallel
with WP-S S.4 part C (one agent slot).
**Phase 1 limits:** no server write path; the page validates and downloads `cad_map.yaml`; saving through `awm_api`
`/cell/model` with role checks comes with T3.1. Contract: `docs/generated/contracts/S10_lite_cell_model.md`,
schema `v2/cad_map.schema.json`.
**Who edits (decided by Truong, 2026-11-03):** Admin edits the cell part of the map (`cell/cad/cad_map.yaml`: robot
base, cell static, positioner axis and permanent rotating set); **Engineer edits the fixture part of the map per job**
(job `fixture.step` / job-level map: fixture, clamps, holders specific to the part, the part itself). The Engineer
cannot change the cell part. Enforced by `awm_api` when saving moves there (T3.1); S10-lite shows which part is being edited.
**Open:** re-matching map entries after a new STEP export (component path + geometric fingerprint, A18 follow-up).
**DES-001 impact:** §4.5 screens table S10 (Phase 1 lite; roles Admin = cell part, Engineer = job fixture part), build
steps UI-1; §7.3 roles table (Engineer: "edit the job fixture role map"); §11 A18 UI; PLAN WP3.

## CP-55 Cell static and rotating geometry are planning-scene objects, not URDF links (decided by Truong, 2026-11-03)

**Finding (WP-S S.4 B, confirmed by review):** in MoveIt 2.12 (Jazzy) `check_state_validity` pads robot links only
against world (scene) objects; self-collisions between URDF links are checked unpadded (`CollisionRequest`
defaults `pad_environment_collisions = true`, `pad_self_collisions = false`; the service cannot change them). With the
Pismo frame, jig, holders and the positioner rotating set as URDF links (D14 wording), the 3 mm padding (F4) and the
clearance-by-padding method (F5) would silently not apply between the head and these objects.
**Decision:** D14 (`awm_description`) holds the kinematics and the robot-mounted geometry only: UR5e from
`ur_description`, LightWELD head / mount / nozzle / conduit, `positioner_joint` with `positioner_axis` /
`positioner_flange` frames (no geometry). Cell static geometry and the positioner rotating set come from the A18 cell
manifest as world collision objects; the rotating set is re-posed with the positioner angle; part / fixture from D19.
**Rules that follow:** (1) a validity / clearance check at positioner angle θ is valid only when the scene reports θ
applied (guard in the loader / checker; review finding 1 of S.4 B); (2) head ↔ UR arm links are checked unpadded —
stated in the TST-002 record; (3) the URDF `positioner_flange` and the loader must use one source for
world_T_positioner_axis.
**DES-001 impact:** §5.1 D14 row ("cell statics … and permanent holders" → kinematics and robot-mounted geometry; static
and rotating cell geometry = scene objects from A18), §5.4 rows Pismo frame / positioner, §4.2 `awm_description` row.

## CP-56 F9 contact rule (b) counts only faces that rise towards the nozzle; spike verdicts reported per nozzle diameter (lead decision, 2026-11-04, delegated by Truong)

**Status:** decided by the lead on 2026-11-04 (Truong asked the lead to decide both points). Applies to the TST-002
Rev 0.3 spike and to M4 (A45) until the IPG nozzle data (I2 / OI-15) arrive.

**Finding:** review of S.5 / S.6 (`ws-s4c-checks` c2fd5bc..a45ca60, M3): F9 rule (b) used the unsigned distance from the
nozzle disc / rim samples to every non-seam face. A face that shares an edge with a seam face and slopes away from the
nozzle (J1 chamfer F11) was flagged as soon as the rim came within s_c of that edge, although the nozzle cannot touch
it without touching the seam faces first. J1 Mode C F9 then failed 100 % at nozzle Ø 9–9.4 mm on F11 alone. With the
assumed Ø 20 flat tip F9 fails 82 % on genuine thread-crest near-contacts (0.014–0.08 mm).

**Decision 1 — rule (b):** a non-seam face fails the contact check only if a part of it within s_c of the nozzle
samples lies on the nozzle side of the contact plane (the plane through the planned contact point, normal to the beam
axis), allowing the contact tolerance. Faces that only fall away from that plane do not count. Thread crests, walls and
ledges that rise towards the nozzle still fail. Rule (a) is unchanged. Reason: F9 must reject a second contact or a
standoff violation that would really happen; a surface below the contact plane cannot be reached by a flat or convex
tip before the seam faces are. The rule stays conservative for anything that protrudes.

**Decision 2 — nozzle geometry is cell data; the planner judges the configured nozzle (Truong, 2026-11-04, replaces
the lead's first wording):** the nozzle (diameter, length, tip length, s_c) is a parameter of the head model in the cell
data (one source: D16 / D14, Admin-owned, part of the hashed context), never a constant in code. M4 plans and checks with
the nozzle that is configured. If a seam fails because of the nozzle (F4 on `nozzle_tip` / `nozzle_body`, F9), the
seam is rejected with the failing points, faces and the cause "nozzle", so the Engineer can change the setup (head
angles, start point, placement) or the cell owner can fit another nozzle (e.g. Ø 12). After a nozzle change the cell
model / context changes, every path planned with the old nozzle becomes stale, and the job is planned again; the STEP
file does not need to be imported again (the part model D02 / D20 is unchanged). The spike verdict uses the configured
TST-002 §3.1 baseline (Ø 20 flat tip); results at Ø 12 / Ø 8 are shown to the meeting only as information on what a
nozzle change would give, not as the verdict. Real nozzle geometry from IPG (I2 / OI-15) stays requested.
Gap found: the nozzle diameter is defined twice today (xacro arg `nozzle_dia_mm` in `lightweld_hbox.xacro` and
`ContactParams.nozzle_radius_mm` in `awm_path.core.contact`) — to be read from one source (T0.1 `cell.yaml`).

**DES-001 / TST-002 impact:** TST-002 Rev 0.3 F9 wording (rule (b) "rises towards the nozzle within s_c"), §11 results
with the per-diameter table; DES-001 §5.4 note on the contact check. Code: `awm_path.core.contact` rule (b), new unit
tests (chamfer sloping away passes; crest rising within s_c fails), S.6 re-run.

## CP-57 GPU pre-filter for the ψ × IK-branch sweep (NVIDIA cuRobo) — proposed, not decided

**Status:** proposal by the lead, 2026-11-05; decision by Truong Hoang / Linh Tran after the TST-002 meeting of
2026-11-06. **Not implemented.** Until decided, speed-up is CPU only (option A, accepted by Truong 2026-11-05: sweep in
parallel worker processes, each with its own `move_group` scene; results identical to the serial run).
**Context:** the sweep is dominated by collision checks (S.5b: 77 min for 12 θ classes, H-BOX). The H-CAD verdict uses
the full CAD mesh of the head (accuracy first, Truong 2026-11-05), which makes each check slower. MoveIt 2 + FCL run on
the CPU only; the IPC / dev PC has an NVIDIA GPU (dev: RTX 2000 Ada Laptop, 8 GB).
**Proposal:** add an optional GPU stage before FCL: cuRobo batched IK and sphere-based collision checking reject
clearly colliding or unreachable (ψ, branch) states; every surviving state is still checked by MoveIt / FCL on the
meshes, and only FCL results enter the verdict (rule 4: 100 % reachable and collision-free, no fallback). The GPU stage
may only remove states that FCL would also reject: its sphere model must never be larger than the link geometry
(spheres inside the link geometry, or a margin test against FCL on a validation set).
**To check before a decision:** cuRobo licence for commercial use (lead has not verified it); CUDA in the Docker image
(ADR-10 image) and on the IPC (DES-001 §8 "NVIDIA GPU optional" would become required for the speed-up only);
agreement test GPU vs FCL on the S.5 / S.5b candidate files (no state rejected by the GPU that FCL accepts).
**DES-001 impact if accepted:** ADR-12 (planner stack: MoveIt 2 + FCL verdict, cuRobo pre-filter), §4.3 step 3, §8
deployment (GPU), Dockerfile.

## CP-58 Goal 1 before Goal 2: weld-path coverage by robot + positioner first, collision second; seam described in the positioner frame (decided by Truong, 2026-11-05)

**Decision 1 — staged goals.** Goal 1 = the robot and the 7th axis (1-axis rotary positioner, 360°, ADR-11) together
follow a complete seam: reachable at every point, correct beam orientation, continuous joints (one IK branch,
|Δq| ≤ Δq_max), joint limits and wrist-3 travel respected, positioner Modes C / B (no simultaneous motion during
emission, ADR-11). Goal 1 is delivered end to end first: STEP import in the Web UI → seam picking → plan → 7-joint
program → simulation playback (Web and mock hardware). **Collision (F4, F5, F9, clearance) is Goal 2** and is not
evaluated in Goal 1. Rule 4 of DES-001 (a seam is accepted only if 100 % reachable *and* collision-free) is **not
relaxed**: Goal 1 results carry `collision_checked: false`, are shown as "kinematic only — collision not checked" in
every UI and report, and are refused by every release gate (no simulation or production release) until Goal 2 has
checked them. The collision work done so far (S.4–S.6, S.5b, S.5c Tasks 0–1, CP-56) is kept on its branches and resumes
as Goal 2.
**Decision 2 — seam in the positioner frame.** After seam picking, M2 writes a derived file `seam_axis.json` next to
`seams.json`: every seam point and its seam frame (X tangent, Z out of the material, beam = −Z) expressed in the
`positioner_flange` frame at θ = 0 (= `positioner_axis` frame at θ = 0), plus per point the radius r from the axis, the
axial position and the angle φ about the axis; header with the SHA-256 of `seams.json`, `placement.json` and
`frames.yaml` (stale when any changes). D04 stays the source in the `part` frame (rule 3); `seam_axis.json` is derived
data, never edited. Planner and program generator use it for θ: a point at θ is the θ = 0 point rotated by θ about the
axis. **Head TCP for Goal 1:** `tool0_T_tcp` from the Fusion head CAD as modelled (Truong 2026-11-05: "keep as CAD":
beam along tool0 −Z, nozzle 40 mm parametric, mount yaw 3.84° as modelled — Assumptions, OI-23).
**DES-001 impact:** §4.3 note on staged goals; §5.1 D04 row (derived `seam_axis.json`); §5.2 job tree; schema
`v2/seam_axis.schema.json`; PLAN.md order (Goal 1 slice across WP1–WP3, Goal 2 = collision tasks).

## CP-59 Synchronised robot + positioner motion (7 axes) in Phase 1 simulation (decided by Truong, 2026-11-05; supersedes the "Phase 1 = Modes B / C only" part of ADR-11 for simulation)

**Decision:** the planner treats the positioner angle θ as a 7th planned joint. For every seam point it chooses
(θ, ψ, IK branch) so that all 7 joints move continuously while the TCP follows the seam at the WPS travel speed
**relative to the part**; the robot and the positioner may move **together during emission** (Option A1 of DES-001
§6.3: one joint trajectory for 7 joints). Mode C (θ moves, robot at rest) and Mode B (θ fixed per arc) remain as special
cases selectable per seam (`positioner_mode: sync | C | B`, default `sync`). Goal 1 (CP-58) is planned and simulated
this way.
**Why:** Truong: "robot and 7th axis computed together to control them synchronously, for a perfect weld path"; it is
the general case — the positioner can keep the weld point in a preferred position (e.g. beam close to vertical, seam
Z close to world +Z) and can cover seams that Mode B with fixed angles cannot (J2, S.5b).
**Cost term (Assumption, parameters in D16):** joint motion of all 7 joints + deviation of the seam Z from world +Z
(weld position) + wrist-3 travel + θ travel; weights recorded with every run.
**Conditions before real hardware (unchanged facts of §6.3, now gates, not deferrals):** (1) the UR5e has no
external-axis support: synchronisation runs in ROS 2 (one controller manager, one 7-joint trajectory, PREEMPT_RT)
— a timing spike must show the path error ≤ the weld tolerance (estimate 0.04 mm at 2 ms jitter, 0.2 mm at 10 ms,
20 mm/s); (2) speed scaling of the UR must scale the positioner identically or stop the weld segment
(`scaled_joint_trajectory_controller` behaviour with a 7th joint to be verified); (3) positioner drive and encoder
(OI-14, OI-25). Until these pass, a synchronised program is valid in simulation only (ADR-17 scope `simulation`).
**DES-001 impact:** ADR-11 text and status; §4.3 step 3 (θ in the state per point), step 5 (time parameterisation
relative to the part with a moving θ); §6.2 (WeldStart / WeldEnd still at rest of robot **and** positioner; the motion
between them may be synchronised); §6.3 Phase 1 decision; §7.1 one 7-joint trajectory controller in simulation; D07 /
D09 schemas (θ per point / per trajectory point, `positioner_mode`).
**Follow-up decisions (Truong, 2026-11-06, after the G1.3 golden run):**
1. *Travel speed* is an Engineer input (UI, D06 / WPS), not a hard output in Goal 1; a low speed is acceptable. The
   requirement that matters now is that robot + positioner follow the seam defined in the STEP. Implementation (lead):
   the planner / program generator report the maximum feasible travel speed per seam from the joint and positioner
   speed limits (no limit raised); a requested speed above it is refused with that number shown — never slowed down
   silently (heat input). G1.3 golden: J1 max 6.98 mm/s, J2 sync max 3.21 mm/s.
2. *Weld position* stays a **soft** preference (weight `w_up` in the cost, D16, Assumption); no hard limit on the
   seam-Z deviation from world +Z in Goal 1. Note: with one horizontal positioner axis J2 cannot be held flat at every
   point (≥ 90° at the two points whose normal is parallel to the axis); the G1.3 run reached 129.8° with `w_up` 0.05.
3. *Priority:* robot + 7th axis completing the seam in the simulation environment (G1.5 → G1.7). Lead defaults kept
   without objection: closure of a 360° seam to its start state is not enforced; Mode C FK tolerance = F8
   (0.05 mm / 0.1°), sync = 0.01 mm / 0.01°.


## CP-60 CAD import split into "cell import" and "job import", components classified by role in the browser; positioner axis picked on the B-Rep (proposed 2026-11-06 — direction decided by Truong, details for approval)

**Decision (Truong, 2026-11-06):** the STEP assembly tree is shown in the browser and the user classifies its
components by role; the import is split into a **cell import** (once per cell) and a **job import** (once per part
number). In S2 the Engineer picks the seams and the **positioner reference (centre of the rotary table)**; the seam file
in the positioner frame (`seam_axis.json`, CP-58) is built from these two inputs for "Plan all seams".
**Why:** today a new job has no placement (manual pose entry in G1.6 is an Assumption); the mechanical team already
structures the Fusion assemblies by device; one source per piece of information (robot / tool / positioner = cell,
fixture / part = job) avoids two files disagreeing about the robot position.
**Roles (decided by Truong 2026-11-06: four roles + default; data, not source).** The roles say only what a component moves with — the planner and the collision check need nothing else:

| Role (UI name) | Id | Moves with | Used for |
|---|---|---|---|
| Robot | `robot` | robot joints | locating the base and the cross-check only; the robot model always comes from `ur_description` (ADR-16), CAD robot geometry is never a collision object |
| Tool (đầu hàn) | `tool` | robot flange | head, mount, conduit: display, collision, TCP teach (CP-61) |
| Rotary fixture (gá xoay) | `rotary_fixture` | positioner θ | everything mounted on the 7th axis that turns while welding (flange, holders, clamps): collision objects moving with θ |
| Part (chi tiết hàn) | `part` | positioner θ | weld solids: seams are picked only here; only the nozzle tip may touch the seam faces (ACM) |
| *(not assigned)* → Static | `static` | nothing | default for every unassigned component: static obstacle (frame, table, positioner base and motor, enclosure, a fixture that does not turn) |

Unassigned = static obstacle is the safe default (a forgotten component is still checked). Removing a component from the computation is an explicit "exclude" action, listed in the cross-check report; it is not a role. A role set on an assembly node applies to its children. The first classification is saved as a template keyed by component instance names; the next import with the same names is pre-classified and only reviewed. The positioner axis is not a role: it is a separate reference pick (below).
**Collision pairs this gives (Goal 2):** robot / tool ↔ static at q; robot / tool ↔ rotary fixture + part at (q, θ) (Mode C over the full turn); nozzle tip ↔ part only on the seam faces; rotary fixture + part ↔ static swept over the positioner θ range (new check: a long part must not hit the base or the frame while turning).
**Cell import (Admin, S10):** STEP with robot, tool, positioner, frame. Outputs: cell `roles.json`, robot base
location, positioner axis (picked circular edge / cylindrical face → centre + axis exactly from the B-Rep; θ = 0 = pose
in the file; `REF_POSITIONER` marker used first if present), tool meshes + TCP (CP-61), cell meshes. Writes the cell
model (`cell/model/frames.yaml`, head folder) — nominal D08 until calibration (A51–A55) replaces it.
**Job import (Engineer, S1 → S2):** STEP with at least the positioner (or its flange), fixture and part; robot / tool
may be present and are then **reference only** (cross-check, never override the cell). Outputs: job `roles.json`,
positioner axis pick in the job file, D19 `placement.json` computed (`positioner_flange_T_part`, `source: job_cad`),
D20 view model with all solids and a role per node (only `part` edges pickable), cross-check report.
**Cross-check (`job/cad_check.json`, shown green / yellow / red in S2; tolerances Assumption, in D16):**
1. job positioner vs cell positioner — picked circle radius, bounding box and fingerprint of `rotary_fixture`;
   mismatch → red, Plan blocked;
2. robot in the job file — base → positioner-axis transform vs the cell (e.g. ≤ 1 mm, ≤ 0.1°) → yellow "layout in the
   file differs from the cell; cell used";
3. tool in the job file — TCP vs the cell TCP → yellow, information;
4. `rotary_fixture` + `part` lie around the picked axis, and sweeping them over the θ range does not hit `static`
   → red;
5. a picked seam edge must belong to `part` solids and lie between two of them → refused at pick time;
6. hashes of the job STEP, `roles.json`, the axis pick and the cell `frames.yaml` recorded in `seam_axis.json` → any
   change makes it stale ("re-plan").
**Frame chain (unchanged CP-58):** robot → positioner axis (cell, later D08) → part (job STEP + roles + axis pick) → seam.
**DES-001 impact:** D01 becomes the job STEP **assembly** (part + fixture + positioner, optional reference robot / tool);
D21 keeps the cell CAD (`cell/cad/`), `job/fixture.step` is dropped (fixture = a role inside D01); new data items
`roles.json` (cell and job, schema `v2/cad_roles`) and `cad_check.json`; D19 gains `source: job_cad`; D20 gains roles;
A15 (place part) becomes "classify + pick positioner axis"; A18 uses the same classification UI; screens S2 and S10;
§5.2 job package; ADR-16 (the `cad_map.yaml` becomes UI-generated `roles.json`); ADR-17 (`job_cad` counts like
`fixture_cad` for a production release). Supersedes the G1.6 manual / copied placement (kept as a test helper only).
**Open for approval:** tolerances of checks 1–2 (proposal ≤ 1 mm, ≤ 0.1°, Assumption); θ = 0 = positioner pose in the
file when there is no `REF_POSITIONER` marker (proposal). Role list decided 2026-11-06.

## CP-61 TCP taught in the browser on the tool CAD; never guessed (proposed 2026-11-06 — direction decided by Truong)

**Decision (Truong, 2026-11-06):** the robot configuration page has a "Teach TCP" function: the user selects the TCP
on the 3D tool model with the mouse and saves it.
**Bug that triggered it (2026-11-06):** the automatic beam estimate of `export-head` (S.5c) took the fiber / cable
connector end behind the flange as the nozzle: TCP (−67.6, 0, −385.4) mm along tool0 −Z. The nozzle tip is at tool0
(43.8, 0, 279.6) mm, beam (0.866, 0, 0.5) — 60° from the flange axis (Truong's mark, located on the barrel axis in the
B-Rep). All Goal 1 results of G1.3 / G1.5 (J1, J2 coverage, 1.19 mm/s) were computed with the wrong TCP and are
re-run. Interim fix: TCP from an explicit map entry (`frames.tcp`), automatic estimates behind the flange refused.
**Teach procedure (proposal):** (1) the user clicks the **circular edge of the nozzle mouth** (or the nozzle's
cylindrical face); (2) the software takes centre and axis from the B-Rep (a mouse click alone gives a surface point,
not the beam axis or its direction), shows the beam arrow pointing out of the nozzle, with a "reverse" button;
(3) the user enters the stick-out / focus offset in mm along the beam (default 0, DES-001 §5.3); (4) save → head.yaml
TCP with `method: teach_ui`, picked edge, offset, user, date, hash; every path planned with the old TCP becomes stale.
**Role / screen:** Admin, S10 "Cell model", tab "Tool / TCP" (§7.3: cell configuration is Admin).
**Relation to calibration:** the taught TCP is the nominal value from CAD; the measured TCP of A51 (4–6 orientations
on the real robot) replaces it in D08 at the hardware stage.
**DES-001 impact:** §5.3 `tcp_laser` source; A51 (nominal vs measured); S10; head.yaml / D08 records.

## CP-62 Recorded Admin override of cross-check 4 for kinematic-only planning (decided by Truong, 2026-11-07)

**Problem:** CP-60 cross-check 4 (`rotary_fixture` + `part` swept over the θ range must not hit `static`) is red for the
golden part 564082 (Jig:1 bearing seats and timing-belt hulls around the pulleys, 4.5–5.1 mm) and stays red by decision
until the mechanical team separates them in the cell CAD (OI-23). The `awm_api` gate refuses seam saving and Plan on a
red check (409 `cad-check-red`), so the Goal 1 acceptance (CP-58: browser import → pick J1, J2 → plan → playback, kinematic
only) cannot be done through the CP-60 flow. The only other path is the manual / copied placement, which ADR-16 and CP-60
keep as a test helper only.
**Decision (Truong 2026-11-07, option B of the Goal 1 audit):** an **Admin** may record an override of **check 4 only**,
for **kinematic-only planning** (`--collision off`, CP-58):
1. Scope: check 4 only. Checks 1 (job positioner ≠ cell positioner) and 5 / 6 stay hard; a stale check still refuses.
2. Recorded, never silent: `job/cad_check.json` keeps `status: red`; the override is a separate record (who, when,
   reason text, the check-4 findings and the `cad_check.json` SHA-256 it applies to). A new or changed `cad_check.json`
   voids it. S2 shows the check red with an "overridden for kinematic planning by <admin>" banner.
3. Effect: seam saving and Plan with collision off are allowed; Plan with collision on (Goal 2) and every release scope
   refuse while check 4 is red, override or not. D07 / D09 record the override (id + hash) next to
   `collision_checked: false`.
4. Roles: only Admin creates or removes the override (ADR-09 roles, §7.3); Engineers see it.
**Why:** check 4 is a collision check (Goal 2 work); Goal 1 is kinematics only and its outputs can never be released
(`collision_checked: false`). The override keeps the CP-60 flow (no hand-typed placement, ADR-16) and keeps the red state
visible until OI-23 is fixed.
**DES-001 impact:** §5.2 job package gains the override record (schema `v2/cad_check_override`, lead); §7.3 Admin rights;
§7.4 `/jobs/{id}/check/override` (POST / DELETE, Admin). No change to release rules (ADR-17).

## CP-63 Admin role includes all Engineer rights (decided by Truong, 2026-11-09)

**Finding (Goal 1 test, 2026-11-09):** logged in as Admin, S2 shows the job read-only ("seam editing is Engineer only,
CP-21") and hides Save roles / axis pick / seams / Plan, so an Admin must log out and log in as an Engineer.
**Decision:** roles are hierarchical: **Admin ⊇ Engineer ⊇ Operator** for viewing and editing. An Admin may do everything
an Engineer may (job roles, axis pick, cross-check, seam picking and editing, process, placement, plan, program, replay,
release request), plus the Admin-only rights of §7.3 (calibration, cell configuration D16, cell model import A18, users,
release approval, CP-62 override).
**Supersedes:** CP-21 "seam picking by Engineer only" (now: Engineer **or Admin**); DES-001 §7.3 table and §4.5 screen
roles column (S2–S5: Engineer, Admin).
**Unchanged:** the record of who did what (`created_by`, override `by`); ADR-17 release approval stays Admin; a release
requested and approved by the same Admin is still recorded with both steps.
**Impact:** `awm_api` role checks (`require('Engineer')` → Engineer or Admin), `awm_web` `canEdit`, role tests, OpenAPI
role notes.

## CP-64 Manual jog page (S11) and taught cell home (D22) (decided by Truong, 2026-11-13)

**Finding / intent:** "go home" and the planner IK seed use a nominal SRDF home (`awm_cell_v2.srdf.xacro` group_state
"home", `initial_positions_v2.yaml`), an Assumption. The user needs a teach-pendant / RViz-like way to move the robot by
hand and teach a home that fits the cell, declared with the cell (same spirit as teach TCP, CP-61).
**Decision:** a new Admin screen **S11 "Manual control (Jog)"** jogs the UR5e (joint + Cartesian) and the positioner
(joint) **in the browser** (model + analytic UR IK/FK in `awm_api` via `UrAnalyticKinematics`; no MoveIt, no controller,
no hardware), offers move-to a typed pose, and **teaches the cell home** (6 robot joints + positioner angle) into
`cell/model/home.json` (new data item **D22**). The Cartesian readout shows both `tcp_laser` and `tool0`; Cartesian jog
acts at `tcp_laser` using the taught cell TCP offset (`head.yaml`, nominal — not D08). New API `/cell/fk`, `/cell/ik`,
`/cell/home` (GET/PUT). Design: `docs/superpowers/specs/2026-11-13-jog-teach-home-design.md`.
**Scope boundary (safety):** simulation only — S11 never opens a ROS control graph or commands mock/URSim/real hardware.
Controller-backed / real-robot jog is OUT of scope and needs its own safety design (enabling device/deadman, reduced
speed, Remote, hardwired E-stop, REQ-002) before it may drive any controller — new open item. The jog page does not
check collisions (no planning scene); joint-limit and reach checks only.
**Supersedes:** the SRDF nominal home as the only home source (consumers that read D22 use it; absent D22 keeps the SRDF
nominal — no change for existing jobs).
**Unchanged:** offline/online split (ADR-05, S11 is a setup tool, not runtime); ADR-13 (driver behind an interface);
units mm/deg (ADR-14); teach home stores joint angles only (frame-independent).
**Follow-up (not in this slice, Truong 2026-11-13):** wire D22 into the planner IK seed (`ChainParams.home_joints_rad`)
and the runtime "go home" (Sim-2 state machine).
**Impact:** §4.5 (new screen S11), §5.1/§5.2 (D22 `cell/model/home.json`), §7.3 (S11 Admin), §7.4 (new endpoints),
§13 (OI: real-robot jog safety; D22 → planner seed / "go home").

## CP-65 Collision granularity of multi-part fixtures: one hull per direct child (proposed 2026-11-13, Truong — for approval)

**Finding (test of 2026-11-13, job with a holder of several parts):** a role assigned to a parent assembly is inherited by
all its children (`awm_core.cad_roles.effective_roles`). For collision, the cell import (A18, `awm_geometry.scene`)
makes **one object per top-most node with the role** and offers two modes in `cad_map.yaml` `collision:`:
`hull` (default — one convex hull over the tessellation nodes of every child solid) or `mesh` (the real solid meshes,
welded per solid and stacked in one STL; no boolean union). A single hull is fast but fills the slots / forks of a
holder that grips the part, so the part or the head may be reported in collision (false red in check 4 or planning);
`mesh` is exact but heavier and needs closed solids. In the job import (`awm_geometry.job_placement`), components that
exist only in the job file get **one hull per leaf**, not per top-most node — inconsistent with the cell import.
**Proposal:**
1. New collision mode **`hull_children`**: one convex hull per **direct child** of the node that carries the role
   (a child that is itself an assembly gives one hull over its whole subtree). All hulls belong to the same scene object
   (same parent frame, same padding, same role), so the fixture stays one entry in D19 / the scene manifest with several
   meshes. No boolean union, no new dependency; meshing stays once per import (cached).
2. The same three modes (`hull`, `hull_children`, `mesh`) for job fixtures, chosen per component in the job roles
   (default `hull_children` for an assembly, `hull` for a single part); job-only components grouped by their top-most
   role node like the cell import (removes the per-leaf rule).
3. S10 / S2 role table: a collision-mode column next to the role (Admin / Engineer), default as above; the cross-check
   and the scene report show the mode and the hull / solid volume ratio so a coarse hull is visible.
**Unchanged:** role inheritance; the part itself keeps its exact mesh with `face_id` (§5.4); padding values (D16);
FCL works on meshes only (§5.4).
**Impact:** §5.4 collision table (fixture / holders: "convex hulls per child, or mesh"), D19 / scene manifest schema
(an object may carry several meshes), `cad_map.yaml` `collision:` values, `awm_geometry.scene` and `job_placement`,
`awm_web` S10 / S2 role table, tests (hull per child, subtree hull, job-only grouping).

# Review of 2026-11-14 (PM / QA / Dev review against the restated output goal)

Record: `docs/generated/doc_review_2026-11-14.md`. Output goal restated by Truong on 2026-11-14: one software product
that uses the robot, the welding source and the rotary positioner to weld any part (frames, pipe sections, …) designed
in 3D with SolidWorks or Fusion.

**Applied on 2026-11-14 (decided items, no design change):** CP-59 and CP-63 written into DES-001 Rev 0.5
(`docs/generated/release_rev0.5/`): §1.2, §1.4, §4.2 M4, §4.3 steps 3 and 5, §4.5 S2–S4, §6.2, §6.3, §7.1, §7.3, ADR-11,
§11 A49, Appendix A; new OI-31 (`sync` on real hardware: timing spike, one speed scaling on 7 joints, PLC rule, lag
monitoring). CP-66…CP-71 below are **proposed, not decided**.

## CP-66 Product goal statement and operating envelope of "any part" (proposed 2026-11-14 — for Truong's decision)

**Finding:** the goal "weld any part designed in SolidWorks / Fusion" is not written as a requirement anywhere
(REQ-001 is only planned; DES-001 §12 says "≥ [TBD] jobs"), and no document states the limits inside which "any part"
holds.
**Proposal:** add to DES-001 §1.2 / §2.1 one goal statement and an operating envelope kept as cell data (D16, Admin):
1. *Input:* one STEP AP214 / AP242 assembly per part number (part solids + fixture + positioner flange, CP-60), exported
   from SolidWorks or Fusion; solids closed; joint gap ≤ the joint-detection tolerance (CP-52, value in D16).
2. *Part envelope:* maximum swing radius about the positioner axis, maximum length along it and maximum mass of
   part + fixture — values from the mechanical design (OI-14, OI-27); checked at job import (extends cross-check 4).
   Fact: the UR5e has 850 mm reach and 5 kg payload; the head (1.3 kg) and mount are on the flange.
3. *Seam classes in Phase 1:* butt, fillet, lap on edges between two part solids, open or closed; pipe-to-pipe saddle
   seams included; single pass; tacks by hand (CP-19).
4. *Outcome per seam:* a path that is 100 % reachable and collision-free, or a rejection with the failing points and
   the colliding objects (rule 4). "Any part" means: no code change per part — not that every seam is weldable.
5. *Not in the goal:* fixture design, distortion control, multi-pass (A46), automatic seam detection (ADR-03).
**DES-001 impact:** §1.2, §2.1 (AD2), §12 Multi-product row; D16; job cross-check. **Needs:** envelope numbers from
mechanical; Truong's confirmation of items 3–5.

## CP-67 Validation part set as the measure of "any part" (proposed 2026-11-14 — for Truong's decision)

**Finding:** every planning result so far is on one part (564082: two closed butt joints, Ø 26.67 mm). Design rule 2
(no part-specific code) is not proven by a second part; no fillet, lap or saddle seam and no SolidWorks export has been
through the chain; E-08 (demo part 564082 or 152056) is open.
**Proposal:** a small part set, each with a job STEP, picked seams and an expected verdict, run by one script with no
source change between parts:
| Part | Purpose | Source |
|---|---|---|
| 564082 (golden) | closed butt joints on tube, coaxial and off-axis | cell CAD (have) |
| Pipe-to-pipe T (saddle) | 3D seam curve, fillet, changing work angle | A25 analytical generator + one CAD model |
| Small tube frame (4 members) | many seams, access between members, sequence and air moves, fillet / T joints | to be supplied (Tac Huynh / mechanical) |
| Plate T / lap joint | open straight seams, lap and fillet | generated test part |
| One of the above exported from SolidWorks | STEP dialect, assembly names, up axis, multi-body weldment | to be supplied |
Acceptance for "multi-product" in simulation: N parts of the set (proposal N = 4, incl. the SolidWorks export) through
import → pick → plan → program → simulation report with zero source changes; every seam ends as PASS or as a rejection
with a cause.
**Impact:** PLAN.md (new tasks), DES-001 §12, test data (the STEP files stay out of git, as today).

## CP-68 Sim-1 / Sim-2 exits and dates (proposed 2026-11-14 — for Truong's decision)

**Finding:** the Sim-1 exit in PLAN.md (J1 Mode C, J2 robot-only, 2026-11-20) predates CP-58 / CP-59. Goal 1 results
are collision off and cannot be released; Sim-1 ends at a simulation release (CP-17), so it cannot close before Goal 2.
Goal 2 has no plan; on 564082 it is blocked by cross-check 4 (OI-23). Sim-2 (2026-12-04) has none of its prerequisites
(runtime packages, URSim in the image, `awm_interfaces` v1, DES-002 weld-event table).
**Options:**
- A (recommended): split Sim-1 — **Sim-1a "kinematic"** = Goal 1 as built (browser import → pick → `sync` plan →
  7-joint program → playback and mock replay, `collision_checked: false`, no release), closed now with a short report;
  **Sim-1b "collision"** = Goal 2 on the golden part + simulation release, new date after the Goal 2 design (CP-70);
  **Sim-2** re-dated from Sim-1b with its own acceptance list. Phase 1 date and deliverable go to Tac Huynh / Linh Tran
  (E-04).
- B: keep 2026-11-20 and 2026-12-04, reduce scope (J1 Mode C only with collision; no J2; Sim-2 = replay of J1 only).
- C: keep scope and dates (not credible per the PM and Dev reviews).
**Also needed:** a due date for OI-23 (cell CAD: bearings / belts separated) before Sim-1b; PLAN.md rebased on
Goal 1 / Goal 2; measurable exit lists moved from the R06 draft into the baseline.

## CP-69 TST-002 outcome and ADR-12 status (proposed 2026-11-14 — for Truong and Linh Tran)

**Finding:** TST-002 Rev 0.3 DRAFT §11 is empty and no Go / Pivot / No-Go was recorded (E-01). Records in
`docs/generated/spike/`: C1 PASS; J2 robot-only C2 FAIL (arc 1 0 %, arc 2 38.3 %), C3 FAIL, C5 557 s (not a valid
measurement per the S.5 review), C8 0 %; no 2- or 3-angle Mode B set covers J2 (best 56.5 %); C4 and C6 not run; all with
the box head model (H-BOX, Ø 20 nozzle, work / travel angle 0). CP-58 / CP-59 then changed the approach.
**Proposal:** record the outcome as **Pivot** (TST-002 §6: C2 fails), with the pivot path actually taken: positioner
as a planned 7th joint (`sync`, CP-59) + real head CAD and taught TCP (CP-61) instead of the box model; the method of
ADR-12 (MoveIt2 + ψ × IK-branch graph) is kept and extended by θ. ADR-12 status → "Kept after Pivot; collision
feasibility on J2 to be shown in Goal 2 (CP-70); Tesseract fallback not started". C4, C6 and a valid C5 move into the
Goal 2 acceptance. Fill §11 of the TST-002 Rev 0.3 draft accordingly (the `docs/reference/` copy is added only from the
Drive master).

## CP-70 Goal 2 design questions: collision with a moving positioner (proposed 2026-11-14 — needs a spec before any code)

**Finding:** DES-001 §4.3 describes collision checking for a static part (ψ × 8 branches per point). With `sync` the
part, holders and fixture move with θ at every point, and:
1. *State space and time:* θ lattice (5°) × ψ (36) × 8 branches per point; the `move_group` service backend measured
   557 s per seam and 77 min for 12 θ classes. No time budget is defined (TST-002 C5 was ≤ 30 s). Options: batched
   in-process FCL backend (PLAN T1.8), kinematic pre-filter then collision only on surviving states, GPU pre-filter
   (CP-57, undecided).
2. *ψ resolution:* ψ step 10° > \|Δq\| ≤ 5° per step, so ψ cannot change along an arc (CP-53 record). Options: ψ step
   ≤ Δq_max near the solution, or continuous ψ refinement; the filler-wire ψ window (B-05) depends on E-05.
3. *Contact and clearance:* confirm the F9 rule as built (no ACM, stand-off, contact by `face_id`; CP-56) against
   DES-001 §5.4 (ACM for `nozzle_tip`); clearances 5 / 10 mm measured on unpadded geometry (R06) — state it in §5.4;
   a head with nozzle length 0 has no contact link.
4. *Between points:* dense check of the 7-joint motion with the rotating set re-posed (rule ≤ 0.02 rad or the
   displacement bound of R06); check that the joint-space spline stays on the seam and holds the TCP speed between
   knots; closing overlap and closure of 360° seams in `sync`; air and index moves collision-checked and stored once.
5. *Data:* `point_status` values `collision`, `clearance`, `nozzle`; CP-65 hull granularity (proposed).
6. *Levers when a seam still fails* (TST-002 §7): work / travel angle sweep, mount tilt, part orientation on the
   fixture, seam start; placement study A17.
**Proposal:** one design spec (`docs/superpowers/specs/`) answering 1–5 with a measurable acceptance on the golden part
(C2 100 %, C4 ≥ 10 mm, C6 placement perturbation, planning time target), reviewed by Truong before a plan is written.

## CP-71 One baseline document (proposed 2026-11-14 — for Truong's decision)

**Finding:** DES-001 Rev 0.5 (working baseline) holds CP-9…CP-23, CP-64 and now CP-59 / CP-63; the Rev 0.6 draft holds
the board decisions B-01…B-25 (stages, ADR-18…23, state space, unpadded clearance, D09 as exact trajectory, no emission
retry) but none of CP-52…CP-65, and was never issued; PLAN.md is built on the draft. OI-29 / OI-30 mean jog safety /
D22 wiring in Rev 0.5 and TST-002 decision / error budget in the draft and in this file (CP-24…35 section).
**Options:** A (recommended) issue one DES-001 Rev 0.6 = draft + CP-52…CP-65 + the accepted parts of CP-66…CP-70, with
the Rev 0.5 OI-29…OI-31 renumbered after OI-44, and DES-003 Rev 0.6 with the impacts listed above (plus D22 and the
CP-60 data items); B keep Rev 0.5 as the baseline and drop the draft, moving only the accepted board decisions into it.
**Also for decision with it:** LaserReady timeout — "one automatic retry" (Rev 0.5 §6.2) or "no retry" (B-17);
B-01…B-25 accepted or vetoed; `CLAUDE.md` refresh (Rev numbers, launch names, base branch, test-skip rule).

## CP-72 Head access map: tool ↔ part accessibility computed in the part frame before the robot search (direction decided by Truong, 2026-11-14 — details for approval)

**Finding:** in the collision-on spike (S.5, S.5b; H-BOX, work / travel angle 0) the dominant blockers on J2 are head ↔
part and head ↔ rotating holders (`part<->nozzle_tip`, `part2_1<->head_body`, `jig_1<->head_body`); s ≈ 2–40 mm has no
valid state at any of 12 positioner angle classes (about 77 min of `move_group` calls). Part and holders turn with the
positioner, so the head ↔ rotating-set check depends only on the head pose relative to the part — not on θ, the IK
branch or the robot. The sweep of DES-001 §4.3 step 3 repeats that check for every θ and branch and cannot report that
the cause is outside the robot.
**Decision (Truong 2026-11-14):** build "level 1" first: per seam point, the set of head orientations (tool yaw ψ, work
angle, travel angle) for which the head set is clear of the part and the rotating fixture (collision, weld clearance on
unpadded geometry, nozzle contact rule F9), computed in `positioner_flange` with an in-process mesh checker; output
`access_map.json` (derived job file) + report with coverage, blocked s ranges and blocking objects / faces, and the
tilt needed. It is a pre-computation in front of the ADR-12 ladder graph (ADR-12 kept); a pose rejected here is
rejected for every θ and IK branch. Design: `docs/superpowers/specs/2026-11-14-head-access-map-design.md`; branch
`g2-1-head-access-map`.
**Planned follow-up levels (not decided, own specs):** 2 = robot + positioner states only for accessible head poses,
fast distance-field checker; 3 = ladder search with a travel-speed (bottleneck) objective, finer ψ, continuous
refinement of the redundant variables with joints from analytic IK; 4 = independent certification of the final
trajectory (exact meshes, displacement-bounded sampling, TCP deviation between knots against the B-Rep seam,
clearance ≥ error budget, stored air moves). Alternatives reviewed 2026-11-14: Tesseract (Descartes + TrajOpt),
cuRobo (CP-57), constrained RRT*, null-space velocity control, learned seeds — choice between own levels 2–3 and
Tesseract deferred until the level 1 result on J2.
**Open for approval (spec §11):** allowed work / travel angle band (welding engineer); nozzle geometry of the real
head (head.yaml `nozzle_len_mm` 0 → F9 refused today, OI-15); collision library in `awm_dev`; whether "not accessible"
blocks Plan; running on a job whose cross-check 4 is red (proposed: allowed, override id recorded).
**DES-001 impact (after approval):** §4.3 new step before step 3; §5.1 / §5.2 new derived file `access_map.json`
(schema `v2/access_map`); §5.4 clearance stated on unpadded geometry; A44 / A45 descriptions; S4 later. Replaces
items 1–3 of the proposed CP-70 for the head ↔ rotating-set part.
