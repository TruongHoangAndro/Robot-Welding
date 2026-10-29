"""Build the DES-001 Rev 0.5 draft from Rev 0.4 by exact, checked text replacements."""
import re
import sys
from pathlib import Path

SRC, DST = Path(sys.argv[1]), Path(sys.argv[2])
s = SRC.read_text(encoding="utf-8")


def rep(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        sys.exit(f"expected {count} match(es), found {n}: {old[:90]!r}")
    s = s.replace(old, new)


def after(anchor, text):
    rep(anchor, anchor + text)


# --- header and history -------------------------------------------------------------------------
rep("| Rev | 0.4 |", "| Rev | 0.5 (draft) |")
rep("| Date | 2026-10-22 |", "| Date | 2026-10-29 |")
rep("| Status | Draft for team review |",
    "| Status | Draft for review — prepared by the software agent from the decisions of 2026-10-27 "
    "(change proposal CP-9…CP-23) and the document review of 2026-10-29; not approved |")
after("D20 view model added; A23 deleted; A88 added | Truong Hoang |",
      "\n| 0.5 (draft) | 2026-10-29 | Decisions of 2026-10-27 and document review of 2026-10-29: (1) 1-axis rotary "
      "positioner in Phase 1, controlled by ROS 2, Modes B / C (ADR-11, 6.3); (2) cell, fixture, head and positioner "
      "geometry from Fusion STEP assemblies with marker components (new ADR-16, A18, D21); (3) placement study A17 and "
      "placement source in D19; (4) Z up, seam / beam axis convention, TCP at the nozzle tip (5.3); (5) release scopes "
      "simulation / production (new ADR-17); (6) calibration before path planning; (7) tack welds by hand; (8) seam "
      "picking by Engineer only; (9) Rev 0.4 inconsistencies corrected (pick_ik, clearance values, wrist-3 limits, "
      "D09 / D10 format, launch names, references, frame chain). OI-13 and OI-20 closed, OI-22…OI-28 added | "
      "Truong Hoang (draft by the software agent) |")

# --- 1 Introduction -----------------------------------------------------------------------------
rep("Phase 1 proof of concept: UR5e on the Pismo frame, **laser welding with an IPG LightWELD 1000** (CW 1000 W, "
    "wobble head, wire feeder), STEP import, manual seam selection in the 3D simulator view, offline path generation, "
    "simulation, calibration, PLC-controlled cell, Web UI. Phase 2 items (MIG process, STL fallback A16, multi-pass "
    "A46, rotary positioner A49 / A55, pause / resume A86, seam tracking A94) are shown only as extension points.",
    "Phase 1 proof of concept: UR5e on the Pismo frame, **laser welding with an IPG LightWELD 1000** (CW 1000 W, "
    "wobble head, wire feeder), **1-axis rotary positioner controlled by ROS 2** (decision 2026-10-27), STEP import, "
    "cell and fixture geometry from the mechanical CAD (Fusion), manual seam selection in the 3D simulator view, "
    "offline path generation, simulation, calibration, PLC-controlled laser, Web UI. Phase 2 items (MIG process, STL "
    "fallback A16, multi-pass A46, continuous 7-axis motion with the positioner, pause / resume A86, seam tracking "
    "A94) are shown only as extension points.")
rep("| [R3] | APLSMN601-DES-003 Function Register Rev 0.2 (source of truth for functions) |",
    "| [R3] | APLSMN601-DES-003 Function Register Rev 0.5 (source of truth for functions) |")
rep("| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.1 |",
    "| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2 (results in Rev 0.3) |")
after("IPG automation interface documentation still to be obtained (OI-2) |",
      "\n| [R12] | Cell CAD `APLSMN601_Automated Weldment System.step` (Fusion export, 2026-10-27): enclosure lower "
      "frame, Pismo plate, UR5e, LightWELD head and mount, 1-axis positioner, jig and holders, golden part "
      "`564082-Assembly-3 parts` |"
      "\n| [R13] | Change proposal DES-001 Rev 0.4 → Rev 0.5 (`docs/generated/DES-001_change_proposal.md` in the "
      "software repository), CP-9…CP-23 |")
rep("| D01…D19 | Data items of the Function Register Rev 0.2 |", "| D01…D21 | Data items of the Function Register Rev 0.5 |")
rep("| Positioner | Rotary table used as 7th (external) axis — Phase 2 slot (ADR-11) |",
    "| Positioner | 1-axis rotary positioner (7th, external axis) carrying fixture and part — Phase 1, controlled by "
    "ROS 2 (ADR-11) |\n"
    "| Mode B / Mode C | Positioner motion modes of Phase 1 (6.3): B = rotate, stop, weld; C = robot at rest, "
    "positioner turns during emission (seam coaxial with the positioner axis) |\n"
    "| Marker component | Small named component in a CAD assembly whose origin and axes define a frame (`REF_*`, "
    "ADR-16) |\n"
    "| Release scope | `simulation` or `production` (ADR-17) |")

# --- 2 Drivers ----------------------------------------------------------------------------------
rep("Tool-yaw DOF must stay free in planning (A44); approach for a positioner kept open as Phase 2 slot (ADR-11) |",
    "Tool-yaw DOF must stay free in planning (A44); 1-axis positioner in Phase 1 for seams coaxial with its axis "
    "(ADR-11, decision 2026-10-27) |")
after("picking and seam editing are core UI functions (Section 4.5) |",
      "\n| AD12 | Cell, fixture, head and positioner designed in Fusion by the mechanical team | Decision 2026-10-27 | "
      "Nominal geometry and frames are imported from the CAD assembly, never typed by hand (ADR-16); measured "
      "calibration replaces nominal values on hardware |")

# --- 3 Context ----------------------------------------------------------------------------------
after("(exact signal set per IPG automation documentation, OI-2) |",
      "\n| Rotary positioner (motor drive) | Joint trajectory from ROS 2 (`awm_positioner`); drive ready / fault "
      "hard-wired to the PLC; STO from the safety chain |"
      "\n| Mechanical design (Fusion) | Supplies cell, fixture, head and positioner CAD as STEP assemblies with marker "
      "components (D21) |")

# --- 4.2 Module catalogue -----------------------------------------------------------------------
rep("place part in the virtual cell (D19); STL fallback with RANSAC cylinder fit. Runs as a separate process "
    "(ADR-15) | A11, A12, A13 (render), A14, A15 · A16 (Phase 2) | 1 / 2 |",
    "place part in the virtual cell (D19, source `study` or `fixture_cad`); import the cell / fixture CAD "
    "assemblies (D21) into the cell model D14 and the nominal calibration (A18); STL fallback with RANSAC cylinder "
    "fit. Runs as a separate process (ADR-15) | A11, A12, A13 (render), A14, A15, A17 (with M4), A18 · A16 "
    "(Phase 2) | 1 / 2 |")
rep("sequence incl. tacks, leg size)", "sequence, leg size; tack welds are made by hand, A81)")
rep("wrist-3 soft limit / cable-wrap check; multi-pass and positioner angle in Phase 2 | A41–A45, A47, A48 · A46, "
    "A49 (Phase 2) | 1 / 2 |",
    "wrist-3 soft limit / cable-wrap check, positioner mode and angles (Modes C / B); multi-pass in Phase 2 | "
    "A41–A45, A47–A49 · A46 (Phase 2) | 1 / 2 |")
rep("error budget, calibration records; positioner axis in Phase 2 | A51–A54 · A55 (Phase 2) | 1 / 2 |",
    "error budget, positioner axis, calibration records | A51–A55 | 1 |")
rep("dry-run and trial-weld records, release workflow and lock |",
    "dry-run and trial-weld records, release workflow (scopes `simulation` / `production`, ADR-17) and lock |")
rep("Replay released trajectory segments on the UR5e through ur_robot_driver, start-state check,",
    "Replay released trajectory segments on the UR5e through ur_robot_driver and on the positioner through its joint "
    "trajectory controller, start-state check,")
rep("`sim` mode emulates PLC + LightWELD with the same interface (ADR-13) |",
    "`sim` mode emulates PLC + LightWELD with the same interface (ADR-13); LightWELD only — the positioner is not "
    "behind the PLC (ADR-11) |")
rep("| A13, A21, A22 (UI), A71 (display),", "| A13, A17, A21, A22 (UI), A71 (display),")
rep("URDF / xacro: UR5e + LightWELD head and mount (incl. fiber / wire-feed conduit section near the head for "
    "collision) + Pismo frame + fixture + clamps, SRDF, MoveIt config; positioner joint added in Phase 2 | D14 | 1 |",
    "URDF / xacro: UR5e (`ur_description`) + LightWELD head and mount (incl. fiber / wire-feed conduit section near "
    "the head for collision) + Pismo frame + positioner joint with its rotating holders + fixture + clamps, SRDF, "
    "MoveIt config; static geometry, head and positioner links generated from the Fusion CAD by A18 (ADR-16) | D14 | "
    "1 |")
rep("Launch files: `sim` (URSim), `real` (UR5e), `workbench`, `web` (awm_api + static front-end)",
    "Launch files: `sim_mock` (Sim-1, mock hardware), `sim_ursim` (Sim-2, URSim + mock positioner), `real` "
    "(UR5e + positioner drive), `workbench`, `web` (awm_api + static front-end)")
rep("| Positioner driver | `awm_positioner` (Phase 2) | ros2_control hardware interface for the rotary table (7th "
    "joint) + simulated joint | A49, A55 (runtime) | 2 |",
    "| Positioner driver | `awm_positioner` | ros2_control hardware interface for the rotary table (7th joint) in "
    "the same controller manager as the UR driver; `mock_components` in simulation; drive on EtherCAT proposed "
    "(OI-14) | A83 (positioner motion), A55 (runtime) | 1 |")
rep("LightWELD driver (trigger, preset, interlocks, wire feeder), safety supervision, alarms |",
    "LightWELD driver (trigger, preset, interlocks, wire feeder), positioner drive ready / fault monitoring "
    "(emission off on fault), safety supervision, alarms |")

# --- 4.3 / 4.4 ----------------------------------------------------------------------------------
rep("this reuses the MoveIt configuration and pick_ik setup of the Tac demo.",
    "this reuses the MoveIt configuration of the Tac demo. IK is enumerated with an analytic UR solver (all 8 "
    "branches, ADR-12); pick_ik returns one solution and is not used for the sweep. Planning uses the active "
    "calibration D08; a calibration change makes D07 stale (plan after calibration).")
rep("seam frame X = tangent, Z = bisector of the face normals (out of the joint), Y = Z × X (A41).",
    "seam frame X = tangent, Z = bisector of the face normals pointing out of the material (up, towards the head; "
    "for a butt joint between tubes n1 = n2 and Z is the surface normal), Y = Z × X; the beam axis is −Z (A41).")
rep("drop states in collision, beyond joint limits or near a wrist / shoulder / elbow singularity (A44, A45).",
    "drop states in collision, beyond joint limits or near a wrist / shoulder / elbow singularity (A44, A45). "
    "With the positioner (A49): seams coaxial with its axis use Mode C (one robot pose, the positioner turns; the "
    "head is checked against the rotating part and holders over the full turn); other seams use Mode B (a "
    "positioner angle per arc segment, searched together with ψ and the IK branch; robot-only first).")
rep("Layered graph (layer = point, node = valid joint state), edge cost",
    "Layered graph (layer = point, node = valid joint state, edges only within one IK branch with |Δq| ≤ 5° per "
    "step), edge cost")
rep("check wrist-3 soft limit and cumulative rotation (A48).",
    "check wrist-3: ±270° absolute soft limit and ±180° travel within one seam (A48). Mode C: positioner speed = "
    "travel speed / seam radius.")
rep("Approach / retract 30–50 mm along the beam axis and OMPL air moves between seams (A43).",
    "Approach / retract 30–50 mm along the beam axis; OMPL air moves between seams from the chosen joint states and "
    "positioner index moves with the robot parked; every segment checked densely (≤ 0.02 rad joint step) (A43).")
rep("start point, head angles or part placement (A15, A22, A42).",
    "start point, head angles or part placement (A15, A17, A22, A42).")
rep("It loads a released program (D10), checks calibration", "It loads a released program (D10; scope `simulation` "
    "only in simulation mode, ADR-17), checks calibration")
rep("by calling M9 for motion and M10 for weld events.", "by calling M9 for motion (robot and positioner) and M10 for "
    "weld events.")

# --- 4.5 UI -------------------------------------------------------------------------------------
rep("cell, fixture and robot shown in the same scene | Engineer; Operator with \"Seam setup\" permission |",
    "cell, fixture, positioner and robot shown in the same scene | Engineer |")
rep("and tool-yaw override per seam, cycle time | Engineer |",
    "and tool-yaw override per seam, positioner mode (B / C) and angles, placement study (candidates with coverage, "
    "A17), cycle time | Engineer |")
rep("| S7 | Calibration | Guided TCP and fixture calibration, error budget, history | Admin |",
    "| S7 | Calibration | Guided TCP, fixture and positioner-axis calibration, error budget, history | Admin |")
after("| S9 | Users | Users, roles, permissions | Admin |",
      "\n| S10 | Cell model | Import the cell / fixture CAD (D21), marker and CAD-map check report | Admin |")
rep("| UI-3 | Simulation S4 with failing-point colours and configuration / tool-yaw override | A44, A45, A71 | R1 |",
    "| UI-3 | Simulation S4 with failing-point colours, configuration / tool-yaw override, positioner | A44, A45, A49, "
    "A71 | R1 |")

# --- 5.1 Data items -----------------------------------------------------------------------------
rep("Data items D01–D20 are those of the Function Register Rev 0.3.",
    "Data items D01–D21 are those of the Function Register Rev 0.5.")
rep("| D07 | Weld path (part frame) | M4 | M6 | `path.schema.json` | job/`path.json` |",
    "| D07 | Weld path (part frame): tool poses in mm, timed joint path incl. positioner angle and mode, per-point "
    "status, D08 hash | M4 | M6, M7 | `path.schema.json` | job/`path.json` |")
rep("| D08 | Calibration data | M5 | M6, M8, M9 |",
    "| D08 | Calibration data (TCP, fixture frame, part offset, positioner axis; `NOMINAL` from the cell CAD until "
    "measured) | M5 (nominal: M1 / A18) | M4, M6, M8, M9 |")
rep("`program.schema.json` (trajectory segments + events)",
    "`program.schema.json`: JSON, ROS 2 joint-trajectory segments (robot 6 joints + positioner) + weld events; "
    "URScript export = fallback")
rep("`release.schema.json` (program rev, hashes, approver, date)",
    "`release.schema.json` (program rev, hashes, scope `simulation` / `production`, approver, date)")
rep("| D14 | Cell model | Design | M4, M7 |", "| D14 | Cell model | M1 (A18) from D21 + `ur_description` | M4, M7 |")
rep("| D19 | Part placement & collision scene | M1 (A15) | M4, M7 | `placement.schema.json` + meshes |",
    "| D19 | Part placement & collision scene | M1 (A15, A17) | M4, M7 | `placement.schema.json` (incl. `source` = "
    "`study` / `fixture_cad`) + meshes |")
after("| D20 | 3D view model | M1 (A13) | UI (S2, S4), M2 | `view.glb` + `topology.json` | job/`cache/view/` |",
      "\n| D21 | Cell & fixture CAD | Mechanical design (Fusion) | M1 (A15, A18) | STEP AP214 / AP242 assembly, mm, "
      "marker components `REF_*` | cell/`cad/` (cell), job/`fixture.step` (fixture) |")

# --- 5.2 Job package ----------------------------------------------------------------------------
rep("│   ├── calibration/              D08  <date>_<id>.json, active.json\n",
    "│   ├── calibration/              D08  <date>_<id>.json, active.json\n"
    "│   ├── cad/                      D21  cell STEP assembly + cad_map.yaml (component → role, up axis)\n"
    "│   ├── model/                    D14  meshes and frames generated by A18\n")
rep("│       ├── placement.json        D19  fixture -> part, orientation, scene objects\n"
    "│       ├── cache/view/           D20  view.glb + topology.json\n",
    "│       ├── placement.json        D19  fixture -> part, orientation, scene objects, source\n"
    "│       ├── fixture.step          D21  fixture assembly (source = fixture_cad)\n"
    "│       ├── cache/                D02  model.brep, collision.stl (part collision mesh with face_id)\n"
    "│       ├── cache/view/           D20  view.glb + topology.json\n")
after("- The repository root is a parameter. Phase 1: a local folder on the IPC, backed up to the shared drive. "
      "Phase 2: database / MES if needed.",
      "\n- A `production` release requires `placement.json` with `source: fixture_cad` (ADR-17).")

# --- 5.3 Frames ---------------------------------------------------------------------------------
rep("| `tcp_laser` | LightWELD focus point / wire tip at nominal focus offset (A51, OI-13); `tcp_mig` reserved for "
    "Phase 2 | D08 (also written to the UR installation) |",
    "| `tcp_laser` | On the beam axis at the nozzle / wire tip, i.e. the contact point on the seam; tool Z = beam "
    "direction; `focus_offset_mm` (D06, default 0) moves the planned point along the beam axis. Assumption until "
    "IPG data (OI-2); `tcp_mig` reserved for Phase 2 | D08 (also written to the UR installation) |")
rep("| `positioner_axis`, `positioner_flange` | Rotary table axis (A55) — Phase 2 only; then `fixture` is a child of "
    "`positioner_flange` | D08 |",
    "| `positioner_axis`, `positioner_flange` | Rotary table axis (A55) and the flange at angle θ; `fixture` and "
    "`part` are children of `positioner_flange` | D08 (nominal from D21) |")
rep("D07 is always in the `part` frame. M6 computes `base_T_tcp = base_T_fixture · fixture_T_part · part_T_pose · "
    "tcp⁻¹` with the active D08 and writes the D08 hash into D09.",
    "**Axis convention (decision 2026-10-27).** Z is up in `world` and `base`. Seam frame: X = travel direction, "
    "Z = bisector of the face normals pointing out of the material, Y = Z × X; the beam axis is −Z. CAD files may "
    "use another up axis (the Fusion cell CAD is Y-up); A18 applies a fixed rotation per file, stated in the CAD "
    "map.\n\n"
    "**Nominal frames from CAD (ADR-16).** `world`, `base`, `positioner_axis`, `fixture`, `part`, `tool0` and "
    "`tcp_laser` are read from marker components `REF_WORLD`, `REF_UR_BASE` (aligned with the UR controller `base` "
    "frame, which is `base_link` rotated 180° about Z in `ur_description`), `REF_POSITIONER`, `REF_FIXTURE`, "
    "`REF_PART`, `REF_TOOL0`, `REF_TCP`.\n\n"
    "D07 is always in the `part` frame. M6 computes `base_T_tcp = base_T_axis · Rz(θ) · flange_T_fixture · "
    "fixture_T_part · part_T_pose · tcp⁻¹` with the active D08 (θ = positioner angle of the point or segment) and "
    "writes the D08 hash into D09.")

# --- 5.4 Collision model ------------------------------------------------------------------------
rep("| ACM: only the TCP link may touch the seam faces |",
    "| ACM: only the `nozzle_tip` link (last 5 mm of the nozzle, TCP at its end) may touch the part; part, fixture "
    "and holders rotate with the positioner |")
rep("| Fixture, clamps | Mechanical STEP (Apillis design) |", "| Fixture, clamps | Fusion CAD (D21) |")
rep("| LightWELD head + mount, fiber and wire-feed conduit near the head | Vendor / measured STEP (OI-15) | Convex "
    "hulls; conduit as capsules |",
    "| LightWELD head + mount, fiber and wire-feed conduit near the head | Head and mount from the Fusion cell CAD "
    "(D21; the head is a surface model) | Convex hulls; nozzle split into `nozzle_tip` / `nozzle_body`; conduit as "
    "capsules |")
rep("| Pismo frame, table | Mechanical STEP | Boxes | — |",
    "| Pismo frame, table | Fusion CAD (D21) | Boxes / convex hulls | — |\n"
    "| Positioner (rotating holders, shaft side) | Fusion CAD (D21) | Convex hulls moving with `positioner_joint` | "
    "3 mm padding; Mode C checked over the full turn |")
after("No additional file type is needed from the customer.",
      "\n\n**Clearance values (D16).** `padding_mm` = 3 (every link except `nozzle_tip`), `weld_clearance_mm` = 5 "
      "(head body, mount and conduit during weld segments; TST-002 C2), `air_clearance_mm` = 10 (approach, retract, "
      "air and index moves; TST-002 C4). Every segment is checked densely (≤ 0.02 rad joint step), not only at "
      "trajectory waypoints.")

# --- 6 States, sequence, positioner -------------------------------------------------------------
rep("| Start ROS2 nodes, connect UR and PLC, load D16 |",
    "| Start ROS2 nodes, connect UR, PLC and positioner drive, reference (home) the positioner, load D16 |")
rep("| READY | Job loaded, robot at home | Wait for cycle start (two-hand / outside guard, door closed) |",
    "| READY | Job loaded, robot and positioner at home | Operator loads and clamps the parts and tacks them by hand "
    "(OI-26); wait for cycle start (outside guard, door closed) |")
rep("| Emission off, wire feed off, controlled stop, alarm text |",
    "| Emission off, wire feed off, controlled stop of robot and positioner, alarm text |")
rep("(LightWELD fault, interlock open, door open, heartbeat loss from IPC)",
    "(LightWELD fault, positioner drive fault, interlock open, door open, heartbeat loss from IPC)")
rep("6. M9 retracts and moves to the next seam (collision-checked air move).",
    "6. M9 retracts and moves to the next seam (collision-checked air move).\n\n"
    "**Mode C (positioner weld, seam coaxial with the positioner axis).** Steps 1–3 as above with the robot at the "
    "weld start pose and the positioner at rest; step 4 is a positioner weld segment executed by M9 (robot at rest, "
    "positioner turns at ω = travel speed / seam radius plus the closing overlap); step 5 starts when the positioner "
    "is at rest again. **Mode B:** the positioner index move to the planned angle is made with emission off, before "
    "step 1. Rule: WeldStart and WeldEnd only while robot **and** positioner are at rest.")
rep("With a positioner (Phase 2) the same scaling must also be applied to the 7th axis or the job stops (ADR-11).",
    "The same rule applies to a Mode C positioner weld segment: it starts only at 100 % (no override on weld "
    "segments). In Phase 1 the robot and the positioner never move together during emission, so no common scaling "
    "is needed; continuous 7-axis motion (Phase 2) must apply one scaling to both or stop (ADR-11).")
rep("LightWELD faults, wire-feeder faults, gas faults and interlock openings are handled by the PLC (emission off)",
    "LightWELD faults, wire-feeder faults, gas faults, positioner drive faults (hard-wired ready / fault) and "
    "interlock openings are handled by the PLC (emission off)")
rep("### 6.3 Positioner synchronisation analysis (Phase 2, ADR-11)",
    "### 6.3 Positioner synchronisation (Phase 1 Modes B / C, Phase 2 continuous motion, ADR-11)")
rep("| Simple; joints between segments |", "| Simple; joints between segments. **Phase 1 (Mode B)** |")
rep("| Covers most round seams |", "| Covers most round seams. **Phase 1 (Mode C)** |")
rep("**Positioner drive.** An EtherCAT servo driven from the IPC inside the same control loop (e.g. "
    "`ethercat_driver_ros2`) suits A1. A drive behind the Beckhoff PLC over ADS is not deterministic and suits A2 or "
    "B only.",
    "**Positioner drive (decision 2026-10-27: ROS 2 controls the positioner).** The drive is a ros2_control hardware "
    "component (`awm_positioner`) in the same controller manager as the UR driver. Proposed: closed-loop drive with "
    "encoder on EtherCAT (CiA 402, `ethercat_driver_ros2`) on its own IPC port, PREEMPT_RT kernel (OI-14). This "
    "serves Modes B / C now and keeps Option A1 open for Phase 2. A drive behind the Beckhoff PLC over ADS was "
    "rejected: not deterministic, and it would block A1. Safety stays in hardware: drive STO from the safety chain; "
    "drive ready / fault hard-wired to the PLC, which switches emission off on a drive fault.")
rep("**Phase 1 check.** The customer parts are pipe parts (elbow, pipe nipple, flange). If the reach study (WBS 2.5) "
    "or TST-002 shows that 360° seams cannot be welded by the robot alone, the positioner moves into Phase 1 by "
    "change request (OI-20).",
    "**Phase 1 decision (2026-10-27).** The mechanical design contains a 1-axis positioner (motor, coupling, shaft, "
    "two timing-belt stages, holders on the jig; horizontal axis). It is part of Phase 1 (OI-20 closed). On the "
    "golden part 564082, joint J1 (pipe nipple ↔ solid) is coaxial with the axis → Mode C; J2 (solid ↔ elbow) is "
    "76.6 mm off the axis with its axis perpendicular to it → robot alone or Mode B. Accuracy: an angular error δ "
    "gives r·δ at the seam; at r = 13.3 mm, 0.1 mm ⇔ 0.43°, so backlash and encoder resolution must be checked "
    "(OI-25).")

# --- 7 Interfaces -------------------------------------------------------------------------------
rep("| `/awm/execute_segment` | action | M9 ← M8 | program rev, segment id, speed override; feedback %, result |",
    "| `/awm/execute_segment` | action | M9 ← M8 | program rev, segment id (robot, positioner, weld or index "
    "segment), speed override (air / index moves only); feedback %, result |")
after("(topic names to be confirmed on the installed driver version) |",
      "\n| `positioner_trajectory_controller/follow_joint_trajectory` | action | `awm_positioner` controller → M9 | "
      "positioner segments (joint `positioner_joint`); state in `/joint_states` |")
rep("| PLC ↔ MIG welder | Phase 2 | — | — |",
    "| PLC ↔ MIG welder | Phase 2 | — | — |\n"
    "| IPC ↔ positioner drive | EtherCAT (CiA 402) through `ethercat_driver_ros2`, own IPC port (proposal, OI-14) | "
    "position set-points, actual position, drive status | Yes (ros2_control cycle, PREEMPT_RT) |\n"
    "| Positioner drive ↔ PLC / safety | Hard-wired | drive ready / fault to the PLC; STO from the safety chain | "
    "Safety function, independent of software |")
rep("Ownership rule: the PLC is the only device that switches the LightWELD. The IPC requests; the PLC decides based "
    "on interlocks.",
    "Ownership rule: the PLC is the only device that switches the LightWELD. The IPC requests; the PLC decides based "
    "on interlocks. The positioner is moved only by ROS 2 (M9); its safe stop is hardware (STO).")
rep("| Operator | Select and run released jobs; start / stop / reset; view status and alarms; seam picking (S2) only "
    "with the \"Seam setup\" permission |",
    "| Operator | Select and run released jobs; load, clamp and tack parts; start / stop / reset; view status and "
    "alarms |")
rep("| Engineer | Create and edit jobs, run simulation and dry run, request release |",
    "| Engineer | Create and edit jobs, pick and edit seams (only this role, decision 2026-10-27), placement study, "
    "run simulation and dry run, request release |")
rep("| Admin | Calibration, cell configuration D16, user management, approve release |",
    "| Admin | Calibration, cell configuration D16, cell model import (A18), user management, approve release |")
rep("| `/jobs/{id}/seams` | GET / PUT | Seam list D04 (picked edge IDs + attributes, mm) | Engineer / Seam setup |",
    "| `/jobs/{id}/seams` | GET / PUT | Seam list D04 (picked edge IDs + attributes, mm) | Engineer |")
after("| `/jobs/{id}/process` | GET / PUT | Process assignment D06 | Engineer |",
      "\n| `/jobs/{id}/placement` | GET / PUT | D19 placement: candidates (`study`) or import from the fixture CAD "
      "(`fixture_cad`) | Engineer |")
rep("| `/jobs/{id}/release` | POST | Release request / approval (M7) | Engineer / Admin |",
    "| `/jobs/{id}/release` | POST | Release request / approval (M7), scope `simulation` or `production` | "
    "Engineer / Admin |")
after("| `/calibration` | GET / POST | Calibration runs and records D08 | Admin |",
      "\n| `/cell/model` | GET / POST | Cell CAD import (D21 → D14), marker check report | Admin |")

# --- 8 Deployment -------------------------------------------------------------------------------
rep("NVIDIA GPU optional, 2 × Ethernet |",
    "NVIDIA GPU optional, 3 × Ethernet (cell LAN, office, EtherCAT to the positioner drive) |")
rep("URSim container for simulation (`sim` launch);", "URSim container for simulation (`sim_ursim` launch; "
    "`sim_mock` for Sim-1);")
rep("The IPC second port connects to the office network for Git and the shared drive only.",
    "The IPC second port connects to the office network for Git and the shared drive only; the third port is the "
    "EtherCAT segment of the positioner drive.")

# --- 9 ADRs -------------------------------------------------------------------------------------
rep("| ADR-06 | **UR5e-only cell model for Phase 1** — remove GP25, rail and gripper from D14 and sequencer; part on a "
    "fixed fixture | Confirmed for Rev 0.3 (2026-10-22) | Fixed Pismo frame in Phase 1 (F10); keep the demo "
    "dual-robot code in a Phase 2 branch. Positioner handled by ADR-11 |",
    "| ADR-06 | **One robot (UR5e) in the Phase 1 cell model** — remove GP25, rail and gripper from D14 and "
    "sequencer; the part sits on the fixture of the 1-axis positioner (ADR-11) | Confirmed for Rev 0.3 (2026-10-22); "
    "updated 2026-10-27 | Fixed Pismo frame in Phase 1 (F10); keep the demo dual-robot code in a Phase 2 branch |")
s_new, n = re.subn(r"\| ADR-11 \| \*\*Rotary positioner as a Phase 2 slot\*\*[^\n]*\n",
                   "| ADR-11 | **1-axis rotary positioner in Phase 1, controlled by ROS 2**: `awm_positioner` ros2_control "
                   "hardware component in the same controller manager as the UR driver, own joint trajectory "
                   "controller; Phase 1 motion Modes C (robot at rest, positioner turns during emission, coaxial "
                   "seams) and B (rotate – stop – weld); continuous 7-axis motion (Options A1 / A2) in Phase 2; "
                   "safety in hardware (STO, drive fault to the PLC). Analysis in Section 6.3 | Decided 2026-10-27 "
                   "(replaces the Phase 2 slot of 2026-10-22) | The mechanical design already contains the "
                   "positioner; 360° seams coaxial with its axis need no robot orbit. PLC-driven positioner rejected: "
                   "not deterministic and it blocks Option A1. Hardware open (OI-14, OI-25) |\n", s)
if n != 1:
    sys.exit("ADR-11 row not found")
s = s_new
rep("| Proposed — decide after TST-002 | MoveIt2 + pick_ik is already in the Tac demo;",
    "| Proposed — decide after TST-002 (29 Oct 2026) | pick_ik (Tac demo) returns one IK solution and is not used for "
    "the sweep; analytic UR IK gives all 8 branches;")
rep("`sim` mode of M10 for PLC + LightWELD, simulated positioner joint (Phase 2)",
    "`sim` mode of M10 for PLC + LightWELD, `mock_components` positioner joint (Phase 1)")
rep("with pythonocc-core 7.9 (conda-forge, Python 3.12)",
    "with pythonocc-core ≥ 7.9 (conda-forge; tested with 8.0.1 / Python 3.10)")
after("Same split suits the Web architecture |",
      "\n| ADR-16 | **Cell, fixture, head and positioner geometry from Fusion STEP assemblies**: read by M1 with OCCT "
      "XCAF (component tree, names, placements); frames from marker components `REF_*`; component → role map kept "
      "as data (`cell/cad/cad_map.yaml`); fixed CAD → `world` axis rotation per file; the UR5e model always from "
      "`ur_description`; no CAD → URDF plug-in | Decided 2026-10-27 | The mechanical team designs in Fusion; one "
      "source for nominal geometry avoids hand-typed transforms. Fusion construction geometry and joint origins are "
      "not exported to STEP, hence marker components. Read test on the first cell CAD: 98 MB in 56 s, names and "
      "placements kept |"
      "\n| ADR-17 | **Release scopes**: `simulation` (after a PASS simulation report only; never valid for "
      "production) and `production` (PASS report, dry run, trial weld, error budget, placement source "
      "`fixture_cad`, Admin approval). The runtime accepts only `simulation` in simulation mode and only "
      "`production` with real hardware | Decided 2026-10-27 | Sim-2 exercises the same release and hash checks as "
      "production without faking A72 / A73 records |")

# --- 11 Traceability ----------------------------------------------------------------------------
rep("| A15 | Place part in virtual cell | M1 + UI-W | R1 |",
    "| A15 | Place part in virtual cell | M1 + UI-W | R1 |\n"
    "| A17 | Study part placement | M1 + M4 + UI-W | R1 (manual candidates), R3 (automatic) |\n"
    "| A18 | Import cell CAD | M1 | R1 |")
rep("| A49 | Coordinate positioner angle | M4 | Phase 2 |", "| A49 | Coordinate positioner angle | M4 | R1 (Modes C / B) |")
rep("| A55 | Calibrate positioner axis | M5 | Phase 2 |", "| A55 | Calibrate positioner axis | M5 | R2 |")

# --- 12 Quality ---------------------------------------------------------------------------------
rep("Class 4 enclosure and LightWELD interlock loops owned by the PLC;",
    "Class 4 enclosure and LightWELD interlock loops owned by the PLC; positioner drive STO in the safety chain;")
rep("Frame chain with calibration D08; part location A53; error budget A54 against the weld tolerance",
    "Frame chain with calibration D08 incl. positioner axis (A55); part location A53; error budget A54 against the "
    "weld tolerance")
rep("| URSim `sim` launch with simulated PLC / LightWELD;", "| `sim_mock` / `sim_ursim` launches with simulated PLC / "
    "LightWELD and mock positioner;")

# --- 13 Open items ------------------------------------------------------------------------------
rep("| OI-13 | TCP definition for the LightWELD head: focus point vs wire tip, nominal focus offset | A42, A51 | "
    "Welding + robot engineer |",
    "| OI-13 | ~~TCP definition~~ — chosen 2026-10-27: TCP at the nozzle / wire tip, focus offset default 0 (5.3); "
    "Assumption until IPG data (OI-2) | A42, A51 | Khang Doan / IPG |")
rep("| OI-14 | Positioner: 1 or 2 axes, payload, servo / driver, protocol (EtherCAT, Modbus, CANopen) — [R9] Q2 | "
    "ADR-11, Phase 2 | [TBD] |",
    "| OI-14 | Positioner drive: driver model, encoder, fieldbus (EtherCAT proposed), payload, angle limits (cables, "
    "belts); 1 axis in Phase 1 (from the CAD) | ADR-11, M9, `awm_positioner` | Mechanical / electrical |")
rep("and clamp geometry for the collision model — TST-002 I2 / I3 | A45, D14 | Mechanical |",
    "and clamp geometry for the collision model — TST-002 I2 / I3. Head, mount, jig and holders are now in the cell "
    "CAD (head = surface model); nozzle tip / TCP point still to confirm | A45, D14 | Mechanical |")
rep("5 (states: PAUSED) and 7 (deployment)", "5 (states: PAUSED, positioner) and 7 (deployment: positioner drive, "
    "EtherCAT port); new figure for the positioner modes")
rep("| OI-20 | Positioner needed in Phase 1? Decide from reach study (WBS 2.5) and TST-002 on the 360° pipe seams | "
    "ADR-11, schedule | Engineering manager |",
    "| OI-20 | ~~Positioner needed in Phase 1?~~ — closed 2026-10-27: yes, 1 axis, ROS 2 control (ADR-11) | — | — |")
after("| OI-21 | LightWELD loan window and on-site date vs first laser weld; Class 4 enclosure ready before first "
      "emission | Schedule, safety | PM |",
      "\n| OI-22 | Contact at the nozzle tip: rigid tip at 0 mm pre-load, compliant (spring) nozzle or force control; "
      "tack beads on the seam | A42, A45, D14 | Welding + robot engineer |"
      "\n| OI-23 | Marker components `REF_*` in the Fusion models; remove the duplicate `ASSY, Welding tool`; state "
      "the up axis of each export | A18, ADR-16 | Mechanical (Nghia Nguyen) |"
      "\n| OI-24 | Upper laser enclosure (Class 4 guard) is not in the cell CAD — needed for collision when the robot "
      "reaches high | A18, D14 | Mechanical |"
      "\n| OI-25 | Positioner accuracy: belt backlash and stiffness, encoder resolution, against 0.43° (0.1 mm at "
      "r 13.3 mm) | A54, A55 | Mechanical |"
      "\n| OI-26 | Manual tack welding: where (inside the Class 4 enclosure?), with which tool, and how it is made "
      "safe | A81, REQ-002 | Welding engineer / safety |"
      "\n| OI-27 | Which holders rotate with the part; positioner angle limits; J2 robot-only or Mode B | A49, D14 | "
      "Mechanical + software |"
      "\n| OI-28 | Function owners in DES-003 are TBD (proposal in the review of 2026-10-29) | DES-003 | Engineering "
      "manager |")

# --- Appendix A ---------------------------------------------------------------------------------
rep("| Pipeline step 0 / 5.8 Positioner axis calibration | Slot | A55, ADR-11 |",
    "| Pipeline step 0 / 5.8 Positioner axis calibration | Adopted (Phase 1) | A55, ADR-11 |")
rep("| Step 2 Place part in virtual cell, reachability | Adopted | A15, D19 |",
    "| Step 2 Place part in virtual cell, reachability | Adopted | A15, A17, D19 |")
rep("| Step 7 / 5.5 Positioner angle, 7-axis coordination, Options A / B | Slot | A49, ADR-11 |",
    "| Step 7 / 5.5 Positioner angle, 7-axis coordination, Options A / B | Adopted for Phase 1 (Modes B / C); "
    "continuous A1 / A2 = Slot | A49, ADR-11 |")

DST.write_text(s, encoding="utf-8")
print("written", DST, len(s), "chars")
