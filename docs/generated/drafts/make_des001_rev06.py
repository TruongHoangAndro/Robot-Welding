"""Build the DES-001 Rev 0.6 draft from the Rev 0.5 release by exact, checked text replacements.

Sources of the changes:
- architecture review of 2026-10-30, change proposal CP-24…CP-35;
- Architecture Review Board of 2026-10-30 (AI reviewer agents; input for the human review by Truong Hoang and
  Linh Tran), decisions B-01…B-25, new CP-36…CP-51, escalations E-01…E-08 (OI-37…OI-44).
  Record: docs/generated/arch_review_board_2026-10-30.md.

Usage (standard library only):
    python3 docs/generated/drafts/make_des001_rev06.py \
        docs/generated/release_rev0.5/APLSMN601-DES-001_Software_Architecture_Rev0.5.md \
        docs/generated/drafts/APLSMN601-DES-001_Software_Architecture_Rev0.6_DRAFT.md
"""
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


def before(anchor, text):
    rep(anchor, text + anchor)


def add_columns(header, titles, values, default=None):
    """Append columns to the markdown table whose header line equals `header` exactly (must exist once).

    `values` maps the first cell of a row to a list of new cell texts; rows not in `values` get `default`
    (a list), or the script stops when `default` is None. Every key of `values` must match a row.
    """
    global s
    lines = s.split("\n")
    idx = [i for i, line in enumerate(lines) if line == header]
    if len(idx) != 1:
        sys.exit(f"table header found {len(idx)} times: {header[:90]!r}")
    i = idx[0]
    lines[i] = header + "".join(f" {t} |" for t in titles)
    lines[i + 1] = lines[i + 1] + "---|" * len(titles)
    used = set()
    j = i + 2
    while j < len(lines) and lines[j].startswith("|"):
        key = lines[j].split("|")[1].strip()
        if key in values:
            cells = values[key]
            used.add(key)
        elif default is not None:
            cells = default
        else:
            sys.exit(f"no value for row {key!r} in table {header[:60]!r}")
        if len(cells) != len(titles):
            sys.exit(f"wrong number of cells for row {key!r}")
        lines[j] = lines[j] + "".join(f" {c} |" for c in cells)
        j += 1
    missing = set(values) - used
    if missing:
        sys.exit(f"rows not found in table {header[:60]!r}: {sorted(missing)}")
    s = "\n".join(lines)


BOARD = "`docs/generated/arch_review_board_2026-10-30.md`"

# --- header and history -------------------------------------------------------------------------
rep("| Rev | 0.5 |", "| Rev | 0.6 (draft) |")
rep("| Date | 2026-10-29 |", "| Date | 2026-11-02 |")
rep("| Status | Accepted by Truong Hoang on 2026-10-29 for team use (Engineering Manager approval pending). Prepared by "
    "the software agent from the decisions of 2026-10-27 (CP-9…CP-23) and the document review of 2026-10-29 |",
    "| Status | Draft for review — prepared by the software agent from the architecture review of 2026-10-30 "
    "(CP-24…CP-35) and the consensus of the Architecture Review Board of 2026-10-30 (decisions B-01…B-25, "
    "CP-36…CP-51; record " + BOARD + "). The board members were AI reviewer agents (roles, not people); their "
    "result is input for the human review by Truong Hoang and Linh Tran. Not approved. Items marked **Proposed** "
    "need a decision; the board escalations E-01…E-08 are open items OI-37…OI-44. Planned issue: 2026-11-06, "
    "together with the TST-002 decision. Working baseline stays Rev 0.5 until this draft is accepted |")
rep("| Approved by | Linh Tran (Engineering Manager) |", "| Approved by | Linh Tran (Engineering Manager) — pending |")
after("function owners and J2 strategy decided 2026-10-29 (OI-27, OI-28) | Truong Hoang (draft by the software agent) |",
      "\n| 0.6 (draft) | 2026-11-02 | (A) Architecture review of 2026-10-30 (CP-24…CP-35): accuracy chain (new ADR-18) "
      "and error budget (12.2); safety concept, software view (12.1); planning runs (new ADR-19); release integrity "
      "(new ADR-20); `awm_api` split (ADR-21, now rejected); Phase 1 positioner interface (new ADR-22, proposed); human "
      "review gate (new ADR-23, proposed); Web API rules (7.4); test strategy and observability (12.3, 12.4); quality "
      "targets; Sim-1 thin slice; OI-29…OI-35. (B) Architecture Review Board of 2026-10-30 — AI reviewer agents, input "
      "for the human review — decisions B-01…B-25 (record " + BOARD + ", CP-36…CP-51; CP-24…CP-29 and CP-31…CP-35 "
      "modified, CP-30 withdrawn): TST-002 decision on 2026-11-06 with frozen inputs; joint-aware seam frames; "
      "planner state space and acceptance metrics; explicit nozzle contact check without ACM exemption; ψ window per "
      "WPS; M4 core behind backends and a runtime dependency rule; one owner per scene object and a shared scene "
      "manifest; D16 and D08 split; immutable per-revision release bundle; one context record; CLI job runner "
      "(ADR-21 rejected, ADR-19 reduced); ordered Sim-1 slice; staged Web UI and contract freeze; delivery stages "
      "Sim-1…HW-2 (new 2.3); D09 as the exact commanded trajectory; typed weld events without automatic retry; "
      "minimal Sim-2 runtime; ADR-22 conformance wording; accuracy chain restructured; dated open items; network "
      "and host security (new 8.1); M11 / M12 deferred; hardware-stage gate items. Amendment of ADR-17 with a trial release "
      "scope (OI-38, E-02 = A, Truong 2026-11-03); proposed amendment pending escalation: named human PLC / safety roles (OI-28, OI-39). "
      "OI-36…OI-44 added; OI-7, OI-8, OI-33, OI-34, OI-35 closed | Truong Hoang (draft by the software agent) |")

# --- references and terms -----------------------------------------------------------------------
rep("| [R8] | ISO 10218-1/-2, ISO 12100, ANSI Z136.1, ISO/IEC/IEEE 12207 |",
    "| [R8] | ISO 10218-1/-2 (edition 2025, incl. cybersecurity), ISO 12100, ANSI Z136.1, ISO/IEC/IEEE 12207, "
    "IEC 62443-3-3, IEC 60825-1, IEC 60825-4 |")
rep("| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2 (results in Rev 0.3) |",
    "| [R10] | APLSMN601-TST-002 Collision Avoidance Feasibility Spike Rev 0.2 (results and frozen inputs in Rev 0.3, "
    "B-01) |")
rep("| [R13] | Change proposal DES-001 Rev 0.4 → Rev 0.5 (`docs/generated/DES-001_change_proposal.md` in the software "
    "repository), CP-9…CP-23 |",
    "| [R13] | Change proposal DES-001 (`docs/generated/DES-001_change_proposal.md` in the software repository): "
    "Rev 0.4 → Rev 0.5 CP-9…CP-23; Rev 0.5 → Rev 0.6 CP-24…CP-35 and, from the Architecture Review Board, "
    "CP-36…CP-51 |\n"
    "| [R14] | Universal Robots `ur_calibration` (part of `ur_robot_driver`): extraction of the factory-calibrated "
    "kinematics of one robot into a `kinematics_config` file for `ur_description` |\n"
    "| [R15] | Architecture Review Board record of 2026-10-30 (" + BOARD + "): AI reviewer agents, decisions "
    "B-01…B-25, escalations E-01…E-08 — input for the human review |")
rep("| ψ (tool yaw) | Rotation of the weld head about its beam axis — redundant DOF used for reach, collision and "
    "singularity avoidance |",
    "| ψ (tool yaw) | Rotation of the weld head about its beam axis — redundant DOF used for reach, collision and "
    "singularity avoidance; redundant only inside the ψ window of the WPS (filler wire side, tip type; B-05) |")
rep("| Release scope | `simulation` or `production` (ADR-17) |",
    "| Release scope | `simulation` or `production` (ADR-17); `trial` (hardware mode, A72 / A73) and `production` = promotion of a trial bundle (ADR-17 as amended 2026-11-03, OI-38) |")
after("| ADS | Beckhoff Automation Device Specification protocol (TCP 48898) |",
      "\n| Error budget | List of all position error sources between the planned TCP pose and the real seam, with their "
      "values and combined result, compared with the allowed seam deviation (12.2) |"
      "\n| Safety function (SF) | Function whose failure can increase a risk (ISO 13849-1); required performance level "
      "PLr from the risk assessment REQ-002 |"
      "\n| Process interlock | Interlock in the standard PLC that protects the part or the process; not credited as a "
      "safety function until REQ-002 assigns a PLr and a safety-rated realisation (12.1) |"
      "\n| Planning run | One execution of the planning CLI for a job, identified by `run_id`, with its run folder, "
      "inputs, software version and result (ADR-19) |"
      "\n| Context record | One record built by `awm_core` with the cell, calibration, robot and software identity of a "
      "plan or program; embedded in D07, D09, D10 and compared at run time (5.2, B-10) |"
      "\n| Release bundle | Frozen copy of all inputs of one released program revision in `programs/<rev>/bundle/` "
      "(ADR-20) |"
      "\n| Delivery stage | Sim-1, Sim-2, HW-1, HW-2 or Phase 2, with entry and exit criteria (2.3) |")

# --- 1.2 scope ----------------------------------------------------------------------------------
after("Automatic seam detection is not part of the product scope (decision 2026-10-22).",
      " Phase 1 deliverable on 2026-12-04 = Sim-2 (2.3) unless OI-40 (escalation E-04) decides otherwise; HW-1 / HW-2 "
      "dates [TBD] in PM-004. Phase 1 demonstration scope per the customer agreement (OI-5, due 2026-11-13).")

# --- 2. drivers and principles ------------------------------------------------------------------
rep("| Explicit frame chain (part → fixture → base → TCP) with calibration data D08 |",
    "| Explicit frame chain (part → fixture → base → TCP) with calibration data D08; robot-specific calibrated "
    "kinematics mandatory for every motion on hardware (ADR-18); no runtime per-part correction in Phase 1; error "
    "budget by direction and mode (12.2) |")
rep("switching to hardware is a launch parameter, not a code change (ADR-13).",
    "switching to hardware is a launch parameter, not a code change (ADR-13). The launch parameter selects drivers; "
    "M8 derives the mode from the connected drivers and refuses a mix (a declared `hil` mode only while the PLC "
    "reports emission disabled by hardware; B-19).")
rep("9. **Thin slice first.** Build one end-to-end slice (one part, one seam, one WPS) through the whole chain before "
    "widening features.",
    "9. **Thin slice first.** Build one end-to-end slice (one part, one seam, one WPS) through the whole chain before "
    "widening features. **Sim-1 slice (CP-34 as modified by B-12, Proposed; order and acceptance in PLAN.md):** "
    "golden part 564082; J1 Mode C first; J2 robot-only, accepted as a 100 % path or a rejection with failing "
    "points; screens S1 (minimal), S2 and S4 (lite) (4.5).\n"
    "10. **Independent human review of safety-relevant work** (Proposed, ADR-23, effective 2026-11-23). Code and data "
    "that can move the robot or positioner, request emission or define what counts as verified (M8, M9, M10, "
    "`awm_positioner`, `awm_interfaces`, the cell part of D16, the A18 / D14 generator including the contact model, "
    "the M4 acceptance rules and the M7 checks) are merged only after review by a named engineer. During Sim-1 "
    "Truong Hoang signs off the diff of the verification-defining files at each acceptance run. The Cell PLC "
    "program needs a named human author, not only a reviewer (B-18, OI-39).")
before("## 3. Context viewpoint",
       "### 2.3 Delivery stages (B-14, Proposed)\n\n"
       "One stage scheme for all documents. R1–R5 of the charter are kept as reference (Section 11). DES-001 changes "
       "only at stage gates: Rev 0.6 issued 2026-11-06 with the TST-002 decision, Rev 0.7 at the Sim-1 exit, Rev 0.8 "
       "at the HW-1 gate. Between gates, changes are CPs with a status.\n\n"
       "| Stage | Content | Entry | Exit | Acceptance run |\n"
       "|---|---|---|---|---|\n"
       "| Sim-1 | Offline chain on mock hardware plus browser (S1 minimal, S2, S4 lite); J1 Mode C, J2 robot-only "
       "(principle 9) | TST-002 decision (OI-29); v2 data contracts frozen 2026-11-04 | Golden-job planner regression "
       "and FK consistency test (planner vs browser) green; simulation release written | Scripted run by Truong Hoang: "
       "import → pick → plan → program → verify → playback, logged in PROGRESS.md |\n"
       "| Sim-2 | Simulation release replayed by M8 / M9 / M10 sim on URSim plus the mock positioner | Sim-1 exit; "
       "`awm_interfaces` v1 frozen; ADR-23 effective 2026-11-23 | Fault-injection subset and commanded-vs-actual on "
       "URSim pass; independent run by the commissioning engineer (OI-39) | Sim-2 acceptance script (B-19) |\n"
       "| HW-1 | Calibration (A51–A55) and dry run (A72), emission blocked by hardware | DES-002; PLC program without "
       "emission by a human author; ADR-23 active; REQ-002 Rev 0.2; IPC and drive installed; UR safety configuration "
       "designed, with its checksum; drive STO wired; hardened image (OI-42); OI-16 / OI-30 values; IK refinement on "
       "D14; independent Sim-2 run passed | Calibration and dry-run records; touch-check decision (12.2) | Runbook-based "
       "dry run by the commissioning engineer |\n"
       "| HW-2 | Trial weld (A73) | LightWELD on site (OI-21); Laser Safety Officer named; Class 4 controlled area and "
       "written SOP; enclosure complete incl. OI-24; fume extraction interlocked; PLC interlock matrix and weld-event "
       "conformance PASS; OI-22 decided | Trial-weld records; production gate (A74) | Trial-weld campaign (release "
       "scope `trial`, ADR-17) |\n"
       "| Phase 2 | MIG, continuous 7-axis motion, pause / resume (A86), automatic Mode B and A17, seam re-match, "
       "M11 / M12 packages, STL fallback | — | — | — |\n\n")

# --- 3. context ---------------------------------------------------------------------------------
rep("| Customer / remote viewer | Views jobs, simulation and reports in a browser (read-only role) |",
    "| Customer / remote viewer | Phase 2; Phase 1 reports via the shared drive (B-13) |")
rep("| Git server | Source code (Rule 4 of the project README) |",
    "| Git server | Source code (Rule 4 of the project README) on an Apillis-controlled private remote with protected "
    "branches and CODEOWNERS (B-23). Until the remote is confirmed private, customer CAD and job data are "
    "git-ignored and nothing is pushed |")

# --- 4. composition -----------------------------------------------------------------------------
after("| L4 Control & field | UR controller, Cell PLC program, LightWELD, safety | UR controller, Beckhoff PLC, devices |",
      "\n\n**Dependency rule (Proposed, B-06).** The runtime packages `awm_cell`, `awm_executor`, `awm_plc_bridge` "
      "and `awm_positioner` depend only on `awm_core`, `awm_interfaces` and the read-only part of `awm_motion` (FK, "
      "D09 → JointTrajectory conversion, override time scaling, start-state check). They never depend on "
      "`awm_path`, `awm_verify` or MoveIt clients. A CI test checks `package.xml` and the Python imports.")
rep("mm ↔ m adapter for third-party ROS packages (ADR-14) | — (shared) | 1 |",
    "mm ↔ m adapter for third-party ROS packages (ADR-14); owns the D16 cell-configuration schema (B-08), the "
    "context record and its compare function (B-10), the record and weld-log writers (B-24) | — (shared) | 1 |")
rep("STL fallback with RANSAC cylinder fit. Runs as a separate process (ADR-15)",
    "STL fallback with RANSAC cylinder fit; joint detection between solids (general fuse; each face keeps its "
    "source-solid id, B-02). Runs as a separate process (ADR-15)")
rep("| M2 Seam Definition | `awm_seam` | Resolve picked edge IDs to exact B-Rep curves,",
    "| M2 Seam Definition | `awm_seam` | Resolve picked edge IDs to exact B-Rep curves (a picked edge on either "
    "solid is resolved to the joint edge; n1 / n2 come from the exterior faces of both solids, B-02),")
rep("| M4 Path Planner | `awm_path` | Seam discretisation and seam frame,",
    "| M4 Path Planner | `awm_path` | Core library behind KinematicsBackend / SceneBackend (B-06): seam "
    "discretisation and seam frame,")
rep("part location (fixed-jig check, touch sensing later), error budget, positioner axis, calibration records |",
    "part location (fixed-jig check, touch sensing later), error budget, positioner axis, calibration records; import "
    "of the robot-specific kinematics (`ur_calibration`, ADR-18); part location in Phase 1 as a go / no-go check with "
    "a typed LOCATE_PART result (B-21); TCP verification A56 (HW-2, B-25) |")
rep("| M6 Program Generator | `awm_program` | Transform path to base frame, build executable program (joint trajectory "
    "segments + weld events), store with revision |",
    "| M6 Program Generator | `awm_program` | Transform path to base frame, build executable program (joint trajectory "
    "segments + weld events), store with revision; writes the exact commanded trajectory (positions, velocities, "
    "accelerations and times per controller, B-15) |")
rep("| M7 Verification & Release | `awm_verify` | Run program in URSim, playback in the Web simulation view, reach / "
    "collision / cycle-time report, dry-run and trial-weld records, release workflow (scopes `simulation` / "
    "`production`, ADR-17) and lock |",
    "| M7 Verification & Release | `awm_verify` | URSim replay = M8 / M9 runtime chain in Sim-2 (B-19); playback in the "
    "Web simulation view, reach / collision / cycle-time report, dry-run and trial-weld records, release workflow "
    "(scopes `simulation` / `trial` / `production`, ADR-17 as amended 2026-11-03) and lock into a frozen per-revision bundle "
    "(ADR-20); rebuilds the scene from the frozen bundle and self-tests it with the C1 pose set before verification; "
    "verifies the controller-interpolated motion at the controller rate (B-06, B-15) |")
rep("alarm handling and recovery, speed-scaling supervision; pause / resume with overlap in Phase 2 |",
    "alarm handling and recovery, speed-scaling supervision; pause / resume with overlap in Phase 2; mode derived from "
    "the connected drivers (B-19); Phase 1 weld log as append-only JSON lines with a start and an end record per seam "
    "(B-24) |")
rep("speed override for air moves only, speed-scaling monitor, feedback |",
    "speed override for air moves only, speed-scaling monitor, feedback; sends D09 points verbatim, override time "
    "scaling only on air / index segments (B-15); Python in Phase 1 (ADR-08) |")
rep("LightWELD only — the positioner is not behind the PLC (ADR-11) |",
    "LightWELD only — the positioner is not behind the PLC (ADR-11); sim mode table-driven from the DES-002 "
    "weld-event behaviour table (B-18) |")
rep("| M11 Weld Logger | `awm_logger` | ",
    "| M11 Weld Logger | `awm_logger` | **Phase 2 package** (B-24); Phase 1 realised by the M8 JSON-lines weld log "
    "and `awm_core` record types. Phase 2 content: ")
rep("| M12 Inspection & Feedback | `awm_inspect` | ",
    "| M12 Inspection & Feedback | `awm_inspect` | **Phase 2 package** (B-24); Phase 1 realised by the record types "
    "inspection / ncr / macro_etch in `awm_core`. Content: ")
rep("| UI backend | `awm_api` (FastAPI + rclpy node) | REST + WebSocket API (7.4), authentication and roles (A88), "
    "command checks, job repository access, calls to M1–M12 |",
    "| UI backend | `awm_api`: FastAPI, one process, no rclpy in the Sim stages; long operations as CLI subprocesses "
    "(ADR-19); ROS bridge decided at HW-1 (ADR-21 rejected) | REST + WebSocket API (7.4), authentication and roles "
    "(A88), command checks, job repository access, one planning-run slot per cell (ADR-19), calls to M1–M12 |")
rep("+ Pismo frame + positioner joint with its rotating holders + fixture + clamps, SRDF, MoveIt config;",
    "+ Pismo frame + positioner joint with its permanent holders, SRDF, MoveIt config (fixture and clamps are per job "
    "in D19, B-07); xacro macros and loader; cell instance data generated by A18;")
rep("— the contract between modules, reviewed by the team before coding |",
    "— the contract between modules, reviewed by the team before coding; contracts frozen per stage (v2 data "
    "contracts on 2026-11-04; `awm_interfaces` v1 at the Sim-1 exit); after a freeze only MINOR additions, otherwise "
    "via CP (B-13, B-17) |")
before("| Positioner driver | `awm_positioner` |",
       "| Motion library | `awm_motion` (module of `awm_path` in Sim-1; package before Sim-2) | Scene assembly from "
       "files (D14 + D19 scene manifest), FK, D09 → JointTrajectory conversion, override time scaling, start-state "
       "check, dense check (B-06) | — (used by M4, M6, M7, M9) | 1 |\n")
rep("positioner drive ready / fault monitoring (emission off on fault), safety supervision, alarms |",
    "positioner drive ready / fault monitoring (emission off on fault), safety supervision, alarms. Owner / author: "
    "named PLC & safety engineer (OI-4; Proposed, OI-39); the software agent drafts requirements only |")

# --- 4.3 engineering chain ----------------------------------------------------------------------
rep("M4 uses the MoveIt2 `move_group` node with the cell model D14 and the collision scene D19 for IK, Cartesian path "
    "and collision services.",
    "The M4 core is a pure-Python library (no rclpy) behind two interfaces (B-06): KinematicsBackend (analytic UR IK "
    "seeds, refinement, FK) and SceneBackend (batch validity, clearance, closest pair). Phase 1 SceneBackend = "
    "`move_group` services with the cell model D14 and the collision scene D19; a batched in-process backend is "
    "added only if C5 > 30 s per seam (measured). `move_group` plans OMPL air moves only; the scene is reset and "
    "hash-checked before each run.")
rep("Y = Z × X; the beam axis is −Z (A41).",
    "Y = Z × X; the beam axis is −Z (A41). Pitch Δs ≤ min(3 mm, Δq_max / rotation rate of the seam frame), about "
    "1.16 mm at r = 13.3 mm (B-03). For joints between separate solids, n1 and n2 are taken from the exterior faces "
    "of the two solids (joint-aware topology, M1, B-02).")
rep("and the 360° closing overlap (A42).",
    "and the 360° closing overlap (A42). The ψ window comes from D06 (B-05).")
rep("3. Per point, sample tool yaw ψ (e.g. 10° steps) × the 8 UR5e IK branches; drop states in collision, beyond "
    "joint limits or near a wrist / shoulder / elbow singularity (A44, A45). With the positioner (A49): seams coaxial "
    "with its axis use Mode C (one robot pose, the positioner turns; the head is checked against the rotating part "
    "and holders over the full turn); other seams use Mode B (a positioner angle per arc segment, searched together "
    "with ψ and the IK branch; robot-only first).",
    "3. Per point, sample tool yaw ψ inside the WPS ψ window (10° steps; 5° in windows narrower than 60°, B-05) × "
    "the 8 UR5e IK branches; analytic IK seeds are refined against the active kinematics (D14) to ≤ 0.01 mm / "
    "0.01°, within the seed's configuration label (B-21, required from HW-1). Drop states in collision, beyond joint "
    "limits, whose contact check fails (5.4, B-04), or near a singularity: |sin q5| ≥ sin ε_w, |sin q3| ≥ sin ε_e, "
    "wrist-centre distance from the base Z axis ≥ d_s (D16; defaults 8.6°, 5°, 50 mm — Assumption, B-03) (A44, "
    "A45). With the positioner (A49): seams whose axis is within 0.05 mm / 0.1° of the positioner axis (D16, "
    "Assumption; residual eccentricity → E10, B-03) use Mode C (one robot pose, the positioner turns; the head is "
    "checked against the rotating part and holders over the full turn plus the overlap, with the contact check); "
    "other seams: robot-only first; Mode B with split points and positioner angles entered by the Engineer (R1); "
    "automatic Mode B search in R3 (B-12).")
rep("node = valid joint state, edges only within one IK branch with |Δq| ≤ 5° per step)",
    "node = valid state (ψ, configuration label computed from the joint values, unwrapped joint vector), edges only "
    "between states with the same configuration label and max |Δq| ≤ 5° on unwrapped joint values; 2π offsets "
    "enumerated within the joint limits at the first layer; in closed seams the overlap connects back to the "
    "first-layer state; ties broken by cost, then ψ index, then label order (B-03))")
rep("check wrist-3: ±270° absolute soft limit and ±180° travel within one seam (A48). Mode C: positioner speed = "
    "travel speed / seam radius.",
    "check wrist-3 on unwrapped values: ±270° absolute soft limit and ±180° travel within one seam (A48). Mode C: "
    "positioner speed = travel speed / seam radius; ramps planned by A47 inside the dwell or the closing overlap "
    "(B-20).")
rep("every segment checked densely (≤ 0.02 rad joint step) (A43).",
    "every segment checked densely: the step is chosen so that no robot or head point moves more than "
    "`max_check_displacement_mm` (D16) between checked states; positioner steps Δθ ≤ 2·(c − margin) / r_max of the "
    "rotating set; M4 and M7 compute the bound independently (B-03) (A43).")
after("the engineer then changes direction, start point, head angles or part placement (A15, A17, A22, A42).",
      "\n\n**Planning runs (ADR-19, Proposed, B-11).** A plan request starts the planning CLI as a subprocess with a "
      "`run_id` and a run folder `jobs/<id>/runs/<run_id>/` (command line, input hashes, context record, seed where "
      "applicable, JSON-lines progress, start / end, exit status OK / FAIL / CANCELLED). One run per cell, held in "
      "memory by `awm_api`; a second request returns `409` with the active `run_id`; job edits are refused during a "
      "run; no queue and no lock files; cancel = SIGTERM, then SIGKILL (atomic writes: a cancelled run leaves no "
      "D07). Before the run the runner checks that `move_group` is up and that the scene hash matches. Weld, "
      "approach and retract segments are identical for identical inputs within one image digest; air moves are "
      "planned once, stored and re-verified. A plan made from a dirty source tree can be released only with "
      "simulation scope. If the measured scene build per run exceeds 10 s (Assumption), the Sim-1 exit decides on a "
      "long-lived ROS 2 action worker behind the same HTTP contract.")
rep("It loads a released program (D10;", "It loads a released program through the active pointer of its scope (D10;")
rep("checks calibration (D08 hash in the program must match the active calibration),",
    "checks the frozen release manifest, the D09 SHA-256 and the context record against the active context (policy "
    "table; B-09, B-10),")

# --- 4.5 user interface -------------------------------------------------------------------------
rep("`awm_api` (FastAPI + rclpy) ⇄ ROS2 services / actions of M1–M12. The browser never calls ROS directly.",
    "`awm_api` (FastAPI) ⇄ CLI subprocesses of M1–M7 in the Sim stages; ROS bridge from HW-1 (B-11). The browser "
    "never calls ROS directly. Browser and planner build the scene from the same D19 scene manifest and the D14 "
    "URDF (B-07).")
rep("geometric fingerprint (used to re-match seams when the STEP revision changes).",
    "geometric fingerprint (used to re-match seams when the STEP revision changes); joint edges with their solid "
    "pair (B-02).")
rep("split, order, joint type; cell, fixture, positioner and robot shown in the same scene |",
    "split, order, joint type; seam frame and beam axis drawn at the start point and every 90° (B-02); cell, fixture, "
    "positioner and robot shown in the same scene |")
rep("points coloured by status (OK / collision / out of reach / near singularity / joint limit), robot configuration "
    "(shoulder / elbow / wrist −1 / 0 / +1) and tool-yaw override per seam, positioner mode (B / C) and angles, "
    "placement study (candidates with coverage, A17), cycle time |",
    "points coloured by status (OK / collision / out of reach / near singularity / joint limit / contact) with the "
    "colliding link / object, positioner mode (B / C) and angles, cycle time. R3: robot configuration (shoulder / "
    "elbow / wrist −1 / 0 / +1) and tool-yaw override per seam, placement study (A17) — edited in JSON until then |")
add_columns("| # | Screen | Main actions | Roles |", ["Stage (B-13)"], {
    "S1": ["Sim-1 (minimal: list, create)"],
    "S2": ["Sim-1"],
    "S4": ["Sim-1 (lite)"],
    "S6": ["HW-1"],
}, default=["CLI / API in Phase 1; screen Phase 2"])

# --- 5. information -----------------------------------------------------------------------------
rep("`seams.schema.json` (curve, face normals n1 / n2, closed / open, attributes, fingerprint)",
    "`seams.schema.json` (curve, face normals n1 / n2 with source-solid ids (B-02), closed / open, attributes, "
    "fingerprint)")
rep("references the LightWELD preset number and its parameters: power, mode, wobble, wire feed, gas)",
    "references the LightWELD preset number and its parameters: power, mode, wobble, wire feed, gas; filler_wire, "
    "wire_side, psi_window_deg, tip_type — Assumption until the welding engineer gives values, B-05)")
rep("per-point status, D08 hash |",
    "per-point status, context record (B-10), `run_id` of the planning run (ADR-19) |")
rep("`NOMINAL` from the cell CAD until measured) | M5 (nominal: M1 / A18) | M4, M6, M8, M9 | `calibration.schema.json` |",
    "`NOMINAL` from the cell CAD until measured) | M5 (nominal: M1 / A18) | M4, M6, M8, M9 | `calibration.schema.json`: "
    "components kinematics, positioner_axis, fixture, tcp (one per tip type), each with its own SHA-256 (B-08, B-25) |")
rep("JSON, ROS 2 joint-trajectory segments (robot 6 joints + positioner) + weld events; URScript export = fallback |",
    "JSON, one segment per controller (robot or positioner) with time_from_start, positions, velocities and "
    "accelerations exactly as commanded, hold values of the other axes, weld events and the context record; "
    "self-contained (B-15, B-09); URScript export = fallback |")
rep("`release.schema.json` (program rev, hashes, scope `simulation` / `production`, approver, date) | job/`release.json` |",
    "`release.schema.json` (program rev, manifest = SHA-256 of every file of the frozen bundle (ADR-20), context "
    "record, scope `simulation` / `trial` / `production`; production carries `promoted_from` = the trial release of the same bundle, approver, date) | "
    "job/`programs/<rev>/release.json` + job/`active_<scope>.json` |")
rep("| D12 | Weld log | M11 | M12 | SQLite table + CSV export |",
    "| D12 | Weld log | M8 (Phase 1), M11 (Phase 2) | M12 | JSON lines, append-only, start and end record per seam "
    "(Phase 1, B-24); SQLite (Phase 2); quality-record fields at HW-2; each record carries the release hash and the "
    "context record |")
rep("| `logs/weld_log.sqlite` |", "| `logs/weld_log.jsonl` (Phase 1), `logs/weld_log.sqlite` (Phase 2) |")
rep("| `inspection.schema.json` |", "| `inspection.schema.json`; record types inspection / ncr / macro_etch (B-24) |")
rep("| M4, M7 | URDF / xacro / SRDF |",
    "| M4, M7 | URDF / xacro / SRDF — cell statics only: UR5e, head / mount / conduit, frame, positioner kinematics "
    "and permanent holders (B-07) |")
rep("| D15 | Simulation / verification report | M7 | M7 (A74) | JSON + PDF / screenshots |",
    "| D15 | Simulation / verification report | M7 | M7 (A74) | JSON + PDF / screenshots; per segment minimum "
    "distance, closest pair and margin (B-06); validity statement: nominal geometry, no dynamics, no contact force, "
    "no heat, safety slow-downs tested only if the UR safety configuration was loaded (B-19) |")
rep("| D16 | Cell & runtime configuration | Design | all runtime modules, M4 (soft limits) | YAML (ROS2 parameters) | "
    "`awm_bringup/config/` |",
    "| D16 | Cell & runtime configuration | Design (cell part: Admin) | all runtime modules, M4 (soft limits) | Cell "
    "part in `cell/config/cell.yaml` (Admin-owned, schema in `awm_core`, hashed into the context record): soft "
    "limits, clearances, padding, acceptance and path-shaping parameters, positioner range, PLC handshake timeouts, "
    "context policy table. Software defaults (solver and service timeouts, retries) in the package share, not "
    "hashed. Robot joint limits only from `ur_description` / D14. No credentials (B-08) | `cell/config/` (cell part), "
    "`awm_bringup/config/` (defaults) |")
rep("| D18 | Part weld report | M11 | M12, customer | PDF / CSV |",
    "| D18 | Part weld report | CLI export (Phase 1), M11 (Phase 2) | M12, customer | CSV (Phase 1); PDF (Phase 2) |")
rep("`placement.schema.json` (incl. `source` = `study` / `fixture_cad`) + meshes",
    "`placement.schema.json` (incl. `source` = `study` / `fixture_cad` and `verified` — false until REF_* markers "
    "(OI-23) and A52 / A53 measurements exist; only simulation scope accepts false, B-12) + part, fixture and clamps "
    "as attached collision objects on `positioner_flange` + scene manifest (objects, parents, transforms, mesh "
    "references, units, scene hash; B-07) + meshes")
after("│   ├── calibration/              D08  <date>_<id>.json, active.json\n",
      "│   ├── config/cell.yaml          D16  cell part (hashed, Admin-owned, B-08)\n")
rep("│       │   └── verify/           D15\n│       ├── release.json          D10\n",
    "│       │   ├── verify/           D15\n"
    "│       │   ├── bundle/           frozen copies at release (ADR-20)\n"
    "│       │   └── release.json      D10\n"
    "│       ├── active_simulation.json    pointer to the released revision (scope simulation)\n"
    "│       ├── active_production.json    pointer to the released revision (scope production)\n"
    "│       ├── runs/<run_id>/        planning-run folders (ADR-19)\n")
rep("so it can be re-matched when the STEP revision changes.",
    "so it can be re-matched when the STEP revision changes. The seam geometry is self-contained in D04 and the "
    "release bundle (B-09). R3 (with A93): tolerance match (Hausdorff ≤ 0.05 mm, same curve type and solid pair); "
    "the Engineer confirms every re-matched seam; the re-match report goes into the bundle (B-25).")
rep("creates a new program revision that must be verified and released again (A93 → A2–A4).",
    "creates a new program revision that must be verified and released again (A93 → A2–A4). Phase 1: a calibration "
    "change triggers an automatic full re-plan and re-verify of all jobs; requalification rules per OI-36 (B-25).")
rep("- A `production` release requires `placement.json` with `source: fixture_cad` (ADR-17).",
    "- A `production` release requires `placement.json` with `source: fixture_cad` and `verified: true` (ADR-17, B-12)."
    "\n- **Schema versions (CP-29 as modified by B-09).** `schema_version` is `MAJOR.MINOR` and atomic writes "
    "(temporary file + rename) are existing behaviour in `awm_core`. A MINOR change only adds optional fields; a "
    "module refuses an unknown MAJOR version. Released bundles are never migrated in place. Migration scripts are "
    "mandatory only after the first job created by an Engineer; before that, test jobs are regenerated. A MAJOR "
    "change of D09 / D10 needs scripted regeneration and re-verification of the active releases before the runtime "
    "update."
    "\n- **One writer per job (B-11).** The active planning run is the job lock; `awm_api` refuses edits while it "
    "runs. No lock files."
    "\n- **Release bundle (ADR-20, B-09).** At release M7 copies D03, D04 + seam geometry, D06, D07, D19 with the "
    "scene manifest, the referenced D05, D15, the records and the cell part of D16 into `programs/<rev>/bundle/`, "
    "writes `release.json` with the SHA-256 of each copy and the context record, and locks the folder. The job "
    "level keeps one active pointer per scope (`active_simulation.json`, `active_production.json`); a simulation "
    "release never displaces a production one. The runtime checks the active pointer, the frozen manifest, the D09 "
    "SHA-256 and the context record. Job-root files stay editable for the next revision. HW-2 gate: states "
    "superseded / revoked, Admin revocation with a reason, append-only `releases.log`."
    "\n- **Context record (B-10).** `awm_core` builds one context record: cell id / rev, D14 hash, cell-config hash, "
    "D08 and component hashes, robot serial or `SIM`, kinematics hash, controller calibration and UR safety "
    "checksums, `ur_installation` block, Git commit with dirty flag, image digest. Later fields are MINOR additions; "
    "the Sim stages fill what exists. D07, D09 and D10 embed it. One compare function and a policy table in "
    "`cell.yaml` decide refuse / re-plan / warn per field.")
rep("and writes the D08 hash into D09.", "and writes the context record (with the D08 hash) into D09.")
rep("The runtime refuses a program whose calibration hash is not the active one (fixes F14).",
    "The runtime refuses a program whose context record differs from the active context where the policy table says "
    "refuse — always for D08, D14 and the cell configuration (fixes F14; B-10)."
    "\n\n**Robot kinematics (ADR-18, B-21).** The nominal `ur_description` kinematics can differ from the real robot "
    "by centimetres (Fact as a bound: `ur_calibration` README [R14]). Calibrated kinematics are mandatory for every "
    "motion on hardware, the dry run included: the factory-calibrated kinematics of the installed UR5e are "
    "extracted with `ur_calibration` and used in D14 for planning, FK checks and the program. Planning calls IK only "
    "through KinematicsBackend; before HW-1 the analytic seeds are refined by damped least squares on the D14 chain "
    "to ≤ 0.01 mm / 0.01° within the seed's configuration label, and the M6 FK acceptance tightens from 0.2 mm to "
    "0.01 mm (tested in Sim-1 with a perturbed kinematics file). The kinematics hash and the controller calibration "
    "checksum are part of the context record. The `ur-analytic-ik` version is pinned. In simulation the nominal "
    "kinematics are used and marked `NOMINAL`."
    "\n\n**Part frame (B-21).** `fixture_T_part` = nominal (D19). A53 in Phase 1 is a go / no-go check; LOCATE_PART "
    "returns a typed result (go / no-go or measured offset; no-op provider in simulation); D09 reserves a `touch` "
    "segment type. No runtime per-part correction in Phase 1 (design rule 1); any correction is an explicit exception "
    "decided by Truong Hoang (OI-43, escalation E-07). The touch-check method is decided at the HW-1 gate with "
    "measured E2, OI-16 and OI-30.")
rep("| `part` | STEP model origin; nominal pose on fixture from fixture design | D14 nominal, D08 offset from A53 |",
    "| `part` | STEP model origin; nominal pose on fixture from fixture design | D19 nominal (B-07); A53 result (go / "
    "no-go in Phase 1, B-21) in D08 |")
rep("| D08 (also written to the UR installation) |",
    "| D08, one TCP component per tip type (B-25); written to the UR installation together with payload / CoG / "
    "mounting (`ur_installation` block in the context record, checked at INIT where the driver exposes it; HW-1) |")
rep("| ACM: only the `nozzle_tip` link (last 5 mm of the nozzle, TCP at its end) may touch the part; part, fixture and "
    "holders rotate with the positioner |",
    "| No ACM exemption (B-04). The nozzle collision geometry stops at a contact standoff s_c (D16) before the contact "
    "point. Contact is checked explicitly: contact point(s) on the seam faces (face_id of n1 / n2) within "
    "tolerance, no other face within s_c. Pre-load per OI-22; with a rigid tip at 0 mm M7 marks 'contact not "
    "assured' and A73 must prove contact. Part, fixture and holders rotate with the positioner |")
rep("| Fixture, clamps | Fusion CAD (D21) | Simplified STL / convex hulls in D14 / D19 |",
    "| Fixture, clamps | Fusion CAD (D21) | Simplified STL / convex hulls in D19 only, attached to `positioner_flange` "
    "(B-07) |")
rep("| Convex hulls; nozzle split into `nozzle_tip` / `nozzle_body`; conduit as capsules |",
    "| Convex hulls; nozzle collision geometry follows the tip shape (Phase 1: Ø 20 mm nozzle of TST-002 §3.1 until "
    "IPG CAD) and stops at the contact standoff, `nozzle_body` padded; conduit as capsules; wire guide as a link that "
    "turns with ψ once OI-15 gives its geometry (B-04, B-05) |")
rep("**Clearance values (D16).** `padding_mm` = 3 (every link except `nozzle_tip`), `weld_clearance_mm` = 5 (head body, "
    "mount and conduit during weld segments; TST-002 C2), `air_clearance_mm` = 10 (approach, retract, air and index "
    "moves; TST-002 C4). Every segment is checked densely (≤ 0.02 rad joint step), not only at trajectory waypoints.",
    "**Clearance values (D16).** `padding_mm` = 3 (every head link; applies to the validity check), "
    "`weld_clearance_mm` = 5 (head body, mount and conduit during weld segments; TST-002 C2), `air_clearance_mm` = 10 "
    "(approach, retract, air and index moves; TST-002 C4), `contact_standoff_mm` (s_c) = 0.3 (Assumption, B-04). "
    "Weld and air clearances are distances on **unpadded** geometry, measured by distance query or by inflating the "
    "head / mount / conduit links by exactly the clearance in a separate validity check; the method is recorded "
    "(B-03). Every segment is checked densely, not only at trajectory waypoints: the step is chosen so that no "
    "robot or head point moves more than `max_check_displacement_mm` (< 2 × (guaranteed clearance − margin)) between "
    "checked states; positioner steps Δθ ≤ 2·(c − margin) / r_max of the rotating set (B-03).")

# --- 6. interaction -----------------------------------------------------------------------------
rep("reference (home) the positioner, load D16 |",
    "reference (home) the positioner, load D16; device-identity / mode check (simulation vs hardware drivers; a mix → "
    "NOT_READY, B-19) |")
rep("| A53: fixed-jig check or touch sensing (later) |",
    "| A53: typed result (go / no-go or measured offset; no-op provider in simulation, B-21); fixed-jig check in "
    "Phase 1, method decided at the HW-1 gate |")
rep("| Operator reset + cause cleared → IDLE |",
    "| Operator reset + cause cleared + home / start-state check → IDLE; Phase 1 recovery is manual with the teach "
    "pendant; interrupted seam → NCR draft (B-19); reverse replay Phase 2 (A86) |")
after("Heartbeat loss acts as the watchdog: the PLC switches emission off without waiting for the IPC.",
      " **Emission follows the welding axis (Proposed, B-17; PLC requirement for DES-002, implemented by the human "
      "PLC author at HW-2).** Emission while the welding axis (robot or positioner, `weld_axis` in WeldStart) is "
      "stationary is allowed only inside the start-dwell window; after motion has started, a stop ends emission at "
      "once (same sequence as WeldEnd); in Mode C, robot motion during emission ends emission; total emission is "
      "capped at the expected duration + margin (D16). Inputs: hard-wired UR safety output 'Robot Moving' and drive "
      "in-motion output (7.2). The heartbeat counter is advanced by M8's state tick and only forwarded by M10; on "
      "loss the PLC switches emission off first, then stops the positioner.")
rep("on timeout one automatic retry, then FAULT.",
    "on timeout no automatic re-trigger: EmissionOff is confirmed, then FAULT (B-17). EmissionLost during a segment → "
    "FAULT and an interrupted-seam record.")
rep("(time-parameterised by M4 from the WPS travel speed; fixes F12).",
    "(time-parameterised by M4 from the WPS travel speed; fixes F12); M9 sends the D09 points verbatim (B-15).")
rep("positioner turns at ω = travel speed / seam radius plus the closing overlap);",
    "positioner turns at ω = travel speed / seam radius plus the closing overlap; ramps planned by A47 and placed "
    "inside the dwell or the closing overlap, B-20);")
after("Air moves may run with override.",
      " Exception (ADR-17 as amended 2026-11-03, B-16): a `trial`-scope dry run with emission disabled by hardware may run "
      "weld segments with override. Weld segments are planned clear of UR safety slow-down zones and reduced-mode "
      "triggers (B-25, HW-1).")
after("J2 (solid ↔ elbow) is 76.6 mm off the axis with its axis perpendicular to it → robot alone or Mode B.",
      " Mode B index moves approach the target angle from one direction (removes belt backlash from E7, B-20).")

# --- 7. interfaces ------------------------------------------------------------------------------
rep("| program rev, segment id (robot, positioner, weld or index segment), speed override (air / index moves only); "
    "feedback %, result |",
    "| program rev, release SHA-256, segment id and segment controller (robot, positioner, weld or index segment), "
    "speed override (air / index moves only); feedback %, result (B-15, B-17) |")
rep("| WeldStart / WeldEnd with seam, WPS, LightWELD preset; result LaserReady / EmissionOff / fault |",
    "| WeldStart(seam, WPS id / rev, preset, segment type, weld_axis, dwell_s, expected_duration_s, ω, release SHA) / "
    "WeldEnd; results LaserReady / EmissionOff / EmissionLost (with time) / fault; typed in `awm_interfaces` v1, "
    "behaviour per the DES-002 weld-event table (B-17, B-18) |")
rep("| planning (Workbench graph only) |",
    "| planning (Workbench graph only): OMPL air moves; validity / IK through the M4 backends (B-06) |")
rep("| EtherCAT (CiA 402) through `ethercat_driver_ros2`, own IPC port (proposal, OI-14) |",
    "| EtherCAT (`ethercat_driver_ros2`) or CANopen CiA 402 (`canopen_402_driver`); every command derived from the "
    "D09 segment (ADR-22, Proposed; OI-14) |")
after("| E-stop, guard door, laser interlock, robot safeguard stop | Safety function, independent of software |",
      "\n| UR5e → PLC (Robot Moving) | Hard-wired UR safety output (Proposed, B-17; DES-002, HW-2) | Robot Moving: "
      "emission tied to welding-axis motion | Yes (PLC) |"
      "\n| Positioner drive → PLC (in motion) | Hard-wired (Proposed, B-17; DES-002, HW-2) | drive in-motion | Yes "
      "(PLC) |"
      "\n| Rotary ground contact on the positioner output shaft | Mechanical / electrical (open design input, Nghia "
      "Nguyen; B-17) | LightWELD part-contact circuit in Mode C | — |")
rep("1. Every motion or process command (start, jog, home, weld)",
    "1. Every motion or process command (start, home, weld; no jog from any UI, B-13)")
rep("3. Status to the UI is pushed at 10–30 Hz over WebSocket — display only, not used for control.",
    "3. Status to the UI is pushed at 10–30 Hz over WebSocket — display only, not used for control.\n"
    "4. Identities in records come from the login only (B-24). Credentials are kept in a git-ignored secrets file "
    "with salted hashes, never in D16 (B-13). A CLI identity is acceptable for simulation scope only.")
after("### 7.4 Web API (`awm_api`, first version)\n",
      "\nRules (CP-32 as modified by B-13): all paths are under `/api/v1`; the OpenAPI document generated by FastAPI "
      "is the contract and is reviewed like `awm_interfaces`; errors use one model (RFC 9457 "
      "`application/problem+json`: type, title, status, detail, plus `job_id` / `seam_id` where relevant); long "
      "operations (plan, import, verify) return `202 Accepted` with a run id and report progress over the "
      "WebSocket. The Stage column gives the first stage of each endpoint; only the Sim-1 subset is built in Sim-1; "
      "functions without an endpoint in their stage run through the CLI.\n")
rep("| `/jobs/{id}/plan` | POST | Request planning (M4) — async, progress over WebSocket | Engineer |",
    "| `/jobs/{id}/plan` | POST | Request planning (M4) — returns `202` + `run_id` (one run per cell, `409` when "
    "busy), progress over WebSocket (ADR-19) | Engineer |\n"
    "| `/jobs/{id}/runs/{run_id}` | GET / DELETE | Planning run: status and result; cancel (SIGTERM of the run "
    "subprocess) | Engineer |")
rep("| select_job, start request, stop, reset, home (→ M8) |",
    "| select_job, start request, stop, reset (→ M8); no jog from any UI; home only as a stored segment with the "
    "physical start button (B-13) |")
rep("| Release request / approval (M7), scope `simulation` or `production` |",
    "| Release request / approval (M7), scope `simulation` or `production`; the release CLI / API refuses "
    "`production` until the HW-2 gate code exists (B-16) |")
add_columns("| Endpoint | Method | Purpose | Role |", ["Stage"], {
    "`/auth/login`": ["Sim-1"],
    "`/jobs` · `/jobs/{id}`": ["Sim-1"],
    "`/jobs/{id}/view`": ["Sim-1"],
    "`/jobs/{id}/seams`": ["Sim-1"],
    "`/jobs/{id}/process`": ["After Sim-1 (CLI before)"],
    "`/jobs/{id}/placement`": ["Sim-1 (GET); PUT after Sim-1"],
    "`/jobs/{id}/plan`": ["Sim-1"],
    "`/jobs/{id}/runs/{run_id}`": ["Sim-1"],
    "`/jobs/{id}/simulation`": ["Sim-1"],
    "`/jobs/{id}/release`": ["Sim-1 (simulation scope)"],
    "`/wps`": ["After Sim-1 (CLI before)"],
    "`/cell/command`": ["HW-1 (S6)"],
    "`/calibration`": ["HW-1 (CLI before)"],
    "`/cell/model`": ["After Sim-1 (CLI before)"],
    "`/reports/{part}`": ["HW-2 (CSV export before)"],
    "`/users`": ["Phase 2 (fixed local accounts before)"],
    "`ws://…/status`": ["Sim-1 (run progress); cell state HW-1"],
})

# --- 8. deployment ------------------------------------------------------------------------------
rep("Docker image for reproducible build |",
    "Docker image for reproducible build; runtime image from the public `ros:jazzy` base, non-root, pinned by digest "
    "(HW-1 entry; Proposed, OI-42, B-23) |")
rep("CI builds all packages and runs unit tests on every merge request |",
    "CI: self-hosted runner (never on the cell LAN) with the image built from `docker/Dockerfile` and the conda OCC "
    "environment mounted read-only; unit, contract and planner regression per merge, `sim_mock` integration nightly; "
    "until the runner exists, `scripts/ci` in `awm_dev` before each merge, logged in PROGRESS.md; demo image for "
    "development only (B-23) |")
after("the third port is the EtherCAT segment of the positioner drive.",
      "\n\n### 8.1 Network and host security (Proposed, B-23; implementation is a HW-1 entry item)\n\n"
      "| Item | Rule |\n"
      "|---|---|\n"
      "| Zones | Cell LAN and office network; the IPC does not route (ip_forward = 0); default-deny host firewall |\n"
      "| Office port | Outbound Git / shared drive only; HTTPS in only if Engineer screens are needed there |\n"
      "| Cell port | UR (RTDE, dashboard, reverse interface from the robot IP), one ADS route to the PLC, HTTPS from the "
      "panel |\n"
      "| ROS | Two ROS domains on the IPC: engineering graph (`move_group`, planning) and cell graph (no MoveIt); "
      "LOCALHOST discovery; no WAN DDS profile |\n"
      "| Image | Runtime image from the public `ros:jazzy` base with an apt snapshot and a micromamba lock for "
      "pythonocc-core 8.0.1; non-root, not privileged, only CAP_SYS_NICE / rtprio (plus EtherCAT access if chosen); "
      "pinned by digest; the demo image never goes to the IPC (OI-42) |\n"
      "| Devices | UR inbound access restricted to the IPC and unused services disabled; one ADS route; LightWELD web "
      "interface not reachable in production; the real ADS client is not installed in the Sim-stage image |\n"
      "| Web | HTTPS only, session expiry, lockout, append-only audit log (login, release request / approval, "
      "calibration activation, D16 / WPS / user changes) |\n"
      "| CI | Self-hosted runner never on the cell LAN; the IPC never runs CI jobs |")

# --- 9. ADRs ------------------------------------------------------------------------------------
rep("| Proposed — decide at G2 |",
    "| To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; review at the Sim-2 exit; B-14) |")
rep("STL only for display / collision meshes | Proposed |",
    "STL only for display / collision meshes | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; "
    "B-14) |")
rep("runtime checks only start state and hashes | Proposed |",
    "runtime checks only start state and hashes | To be confirmed as Decided on 2026-11-06 (built and tested in "
    "Rev 0.2; B-14) |")
rep("| ADR-07 | **File-based job repository + SQLite weld log** | Proposed |",
    "| ADR-07 | **File-based job repository**; weld log as JSON lines in Phase 1, SQLite in Phase 2 (B-24) | "
    "Repository: to be confirmed as Decided on 2026-11-06 (B-14); weld log: Proposed |")
rep("| ADR-08 | **Python for engineering modules, C++ for real-time-relevant ROS2 nodes (M9)** | Proposed | Python is "
    "fastest for OCCT / geometry work; executor reuses welder_action_server C++ code |",
    "| ADR-08 | **Python for all Phase 1 nodes including M9** (the real-time loop is in ros2_control); C++ only on "
    "measured need | Proposed (B-19); depends on the PLC requirement 'emission follows the welding axis' (B-17) | "
    "Python is fastest for OCCT / geometry work; M9 only sends stored segments and monitors; the demo "
    "welder_action_server mixes planning and execution (F01, F12) |")
rep("`mock_components` positioner joint (Phase 1) | Proposed |",
    "`mock_components` positioner joint (Phase 1) | To be confirmed as Decided on 2026-11-06 (built and tested in "
    "Rev 0.2; B-14) |")
rep("with `awm_api` and M2 | Proposed |",
    "with `awm_api` and M2 | To be confirmed as Decided on 2026-11-06 (built and tested in Rev 0.2; B-14) |")
rep("| Proposed — decide after TST-002 (29 Oct 2026) |",
    "| Proposed — decided at the TST-002 meeting on 2026-11-06 (OI-29). Inputs frozen per TST-002 Rev 0.3 §2 / §5. "
    "Tesseract fallback evaluated after Sim-2, not inside Sim-1 (B-01) |")
rep("adds a second planning stack [R9 5.6] |",
    "adds a second planning stack [R9 5.6]. The planner core sits behind KinematicsBackend / SceneBackend, so "
    "Tesseract stays a drop-in fallback (B-06). ψ freedom depends on the WPS; TST-002 reports free-ψ and wire-window "
    "cases (B-05) |")
rep("risk of unit mistakes is contained by the single adapter and unit tests on it |",
    "risk of unit mistakes is contained by the single adapter and unit tests on it. One adapter per language: "
    "`awm_core` (Python) and the `awm_web` units module (TypeScript), both tested on shared fixtures (B-07) |")
rep("no CAD → URDF plug-in | Decided 2026-10-27 |",
    "no CAD → URDF plug-in; component mapping is default-deny: every STEP component is mapped to a role or to "
    "`ignore`, an unmapped component fails the import (B-07) | Decided 2026-10-27; default-deny Proposed (B-07) |")
rep("| Decided 2026-10-27 | Sim-2 exercises the same release and hash checks as production without faking A72 / A73 "
    "records |",
    "| Decided 2026-10-27; **amended 2026-11-03 (B-16, OI-38, escalation E-02 = option A, Truong Hoang):** scopes simulation / trial / "
    "production. `trial`: hardware mode only, authenticated Engineer; dry run when the PLC reports emission disabled "
    "by hardware (override allowed on all segments), trial weld at 100 %; parts marked TRIAL in D12. `production`: "
    "promotion of the same frozen bundle (same D09 SHA) after PASS trial records and the A74 gate checks. The "
    "runtime accepts `simulation` only in simulation mode, `trial` / `production` only in hardware mode. Before "
    "Sim-2 the M8 scope check is a data table; until the HW-2 gate code exists the release CLI refuses `production` | Sim-2 "
    "exercises the same release and hash checks as production without faking A72 / A73 records. Without a trial "
    "scope A72 / A73 cannot run through M8 / M9 (production needs their records; A72 at reduced speed conflicts with "
    "the 100 % rule) |"
    "\n| ADR-18 | **Accuracy chain**: on hardware, planning and programs use the factory-calibrated kinematics of the "
    "installed UR5e (`ur_calibration`), mandatory for every motion including the dry run; IK only through "
    "KinematicsBackend with refinement on D14 before HW-1; kinematics hash and controller calibration checksum in "
    "the context record; no runtime per-part correction in Phase 1 (design rule 1) — any correction is an explicit "
    "exception decided by Truong Hoang (OI-43, E-07); the error budget (12.2) decides the part-location method at "
    "the HW-1 gate | Proposed (CP-24 as modified by B-21) | Nominal kinematics may be off by centimetres (Fact as a "
    "bound, [R14]); offline programming from CAD depends on absolute accuracy, not repeatability; a runtime IK "
    "re-solve would contradict ADR-05 |"
    "\n| ADR-19 | **Planning runs as CLI subprocesses**: `run_id`, run folder, one run slot per cell (`409` when "
    "busy), job edits refused during a run, cancel = SIGTERM; weld / approach / retract segments identical for "
    "identical inputs within one image digest; air moves stored and re-verified (4.3) | Proposed (CP-28 as modified "
    "by B-11) | One Engineer in Phase 1: no queue and no lock files needed; reuses the tested CLIs with crash "
    "isolation; a byte-equal D07 cannot pass (headers carry `created_at`, OMPL is random). Long-lived action worker "
    "decided at the Sim-1 exit on measured scene-build time |"
    "\n| ADR-20 | **Immutable per-revision release bundle** with manifest and one active pointer per scope (5.2); "
    "the runtime checks the active pointer, the frozen manifest, the D09 SHA-256 and the context record | Proposed "
    "(CP-29 as modified by B-09) | A manifest over editable job files breaks the running release when the next "
    "revision is edited (observed in the Rev 0.2 chain); `release.json` was one job-level file that a new release "
    "overwrote. Alternative: Git-backed job repository (Phase 2 option) |"
    "\n| ADR-21 | ~~**`awm_api` split**: FastAPI gateway and a separate rclpy bridge node~~ | **Rejected for Phase 1** "
    "(board 2026-10-30, CP-30 withdrawn); revisit at HW-1 when S6 needs live state | All three board members: a "
    "second process and an undefined internal protocol with no Phase 1 benefit; no Sim-stage endpoint needs live "
    "ROS data; replaced by one FastAPI process with CLI subprocesses (ADR-19) |"
    "\n| ADR-22 | **Phase 1 positioner interface sized for Modes B / C only**: any drive mode is allowed if "
    "`awm_positioner` derives every command from the D09 segment (no profile set by hand on the drive), the mock "
    "and M7 use the same generator, M9 checks the encoder angle and the ω ripple during emission against D16 limits "
    "(Assumption until OI-16), and drive in-motion / fault are hard-wired to the PLC (STO from the safety chain). "
    "EtherCAT or CANopen CiA 402. EtherCAT with Option A1 stays the Phase 2 target. Detail of ADR-11 — ROS 2 "
    "control of the positioner is unchanged | Proposed (CP-33 as modified by B-20) — decide with OI-14 by "
    "2026-11-20 | What is verified (D09 with planned ramps) is what the axis does, while the fieldbus stays free; "
    "the interface rule constrains the drive purchase, so it comes first |"
    "\n| ADR-23 | **Human review gate**: named human reviewers (CODEOWNERS, protected branch on the company remote) "
    "for M8, M9, M10, `awm_positioner`, `awm_interfaces`, the cell part of D16, the A18 / D14 generator including "
    "the contact model, the M4 acceptance rules and the M7 checks; the PLC program has a named human author; the "
    "software agent prepares, a human approves the merge | Proposed (CP-35 as modified by B-18) — effective "
    "2026-11-23 (Sim-2 start); reviewers named by 2026-11-20 (OI-32); during Sim-1 Truong Hoang signs off "
    "verification-defining diffs; prerequisite: protected branch on the company remote (B-23) | All functions are "
    "owned by the software agent (OI-28); a Class 4 laser cell needs an independent check of every change that can "
    "move axes, request emission or change what counts as verified |")
add_columns("| ADR | Decision | Status | Alternatives / reason |", ["Decide by (B-14)"], {
    "ADR-01": ["2026-11-06"], "ADR-02": ["2026-11-06"], "ADR-05": ["2026-11-06"], "ADR-07": ["2026-11-06"],
    "ADR-08": ["Sim-1 exit (before WP4)"], "ADR-12": ["2026-11-06 (OI-29)"], "ADR-13": ["2026-11-06"],
    "ADR-15": ["2026-11-06"], "ADR-16": ["Decided; default-deny 2026-11-06"],
    "ADR-17": ["Decided; amended 2026-11-03 (OI-38)"], "ADR-18": ["HW-1 gate"], "ADR-19": ["2026-11-06"],
    "ADR-20": ["2026-11-06"], "ADR-21": ["— (rejected)"], "ADR-22": ["2026-11-20 (with OI-14)"],
    "ADR-23": ["2026-11-20 (OI-32)"],
}, default=["Decided"])

# --- 11. traceability ---------------------------------------------------------------------------
rep("| A17 | Study part placement | M1 + M4 + UI-W | R1 (manual candidates), R3 (automatic) |",
    "| A17 | Study part placement | M1 + M4 + UI-W | After Sim-1 (manual), R3 (automatic) |")
rep("| A18 | Import cell CAD | M1 | R1 |",
    "| A18 | Import cell CAD | M1 | Sim-1 scripted import, default-deny mapping; marker checks HW-1 |")
rep("| A49 | Coordinate positioner angle | M4 | R1 (Modes C / B) |",
    "| A49 | Coordinate positioner angle | M4 | R1 (Mode C; Mode B with entered angles), R3 (automatic Mode B) |")
after("| A55 | Calibrate positioner axis | M5 | R2 |",
      "\n| A56 | Verify TCP (new, Proposed, B-25) | M5 | R2 |")
rep("| A85 | Log weld data | M11 | R2 |", "| A85 | Log weld data | M8 (Phase 1), M11 (Phase 2) | R2 |")
rep("| A87 | Export part report | M11 | R2 |", "| A87 | Export part report | CLI (Phase 1), M11 (Phase 2) | R2 |")
before("| Function | Name | Module | Release |",
       "Stage column (B-14): first delivery stage of each function (2.3); the DES-003 Rev 0.6 'Sim stage' column is to "
       "be aligned with it.\n\n")
stage = {
    "A16": "Phase 2", "A17": "After Sim-1", "A18": "Sim-1 (scripted)", "A31": "Sim-1 (sample WPS)",
    "A32": "Sim-1 (sample WPS)", "A33": "Sim-1 (CLI)", "A34": "HW-2", "A46": "Phase 2",
    "A49": "Sim-1 (Mode C; Mode B entered)", "A51": "HW-1", "A52": "HW-1", "A53": "Sim-2 (stub); HW-1",
    "A54": "HW-1", "A55": "HW-1", "A56": "HW-2", "A63": "Sim-1 markers; Sim-2 M10 sim", "A72": "HW-1",
    "A73": "HW-2", "A74": "Sim-1 (simulation scope); HW-2 (production)", "A81": "Sim-2 (minimal)",
    "A82": "Sim-2 (minimal)", "A83": "Sim-2", "A84": "Sim-2 (minimal)", "A85": "Sim-2 (JSON lines); HW-2 fields",
    "A86": "Phase 2", "A87": "HW-2", "A88": "Sim-1 (local login)", "A91": "HW-2", "A92": "HW-2",
    "A93": "Phase 2 (R3)", "A94": "Phase 2", "A95": "HW-2",
}
add_columns("| Function | Name | Module | Release |", ["Stage"], {k: [v] for k, v in stage.items()}, default=["Sim-1"])

# --- 12. quality attributes and new subsections -------------------------------------------------
rep("| Correctness | Offline 100 % path check, released = executed, hash checks | M7 report; FAT |",
    "| Correctness | Offline 100 % path check, released = executed (D09 as the exact commanded trajectory, B-15), "
    "frozen release bundle and context record | M7 report; FAT |")
rep("| Security | Roles Operator / Engineer / Admin; all commands through M8 / backend; physical start confirmation "
    "(7.3) | Role test cases in FAT |",
    "| Security | Roles Operator / Engineer / Admin; all commands through M8 / backend; physical start confirmation "
    "(7.3); network and host security (8.1) | Role test cases and port scan in FAT |")
rep("| Maintainability | One config source, no absolute paths, packages with single responsibility | Code review "
    "checklist |",
    "| Maintainability | One config source, no absolute paths, packages with single responsibility; runtime "
    "dependency rule (4.1) | Code review checklist; dependency test in CI (B-06) |")
rep("| Weld quality | WPS travel speed held in the part frame (A47), speed-scaling rule (6.2), emission-on check | "
    "Speed error < 5 %; macro-etch to the chosen standard (OI-12) |",
    "| Weld quality | WPS travel speed held in the part frame (A47), speed-scaling rule (6.2), emission-on check | "
    "TCP speed of the interpolated D09 trajectory within ±5 % outside the ramps (B-15); macro-etch to the chosen "
    "standard (OI-12) |"
    "\n| Reproducibility | Run record per planning run, deterministic DP tie-break (ADR-19, B-03) | Identical weld / "
    "approach / retract payload for identical inputs within one image digest (max abs Δq ≤ 1e-9 rad, same per-point "
    "status, header excluded from the hash); air moves: same verdict, cycle time ±10 % (B-11) |"
    "\n| Performance | One run slot per cell, ψ × branch sweep behind SceneBackend | ≤ 30 s per seam (TST-002 C5, "
    "recorded with the backend used); ≤ 5 min per golden-part job (Assumption, to be confirmed after TST-002) |"
    "\n| Integrity | Frozen release bundle and context record (ADR-20, 5.2), atomic writes, one writer per job | "
    "Runtime refuses a bundle with one changed byte (test) |"
    "\n| Observability | Version stamps and `run_id` in logs (Sim stages); structured logs and `/diagnostics` from "
    "HW-1 (12.4) | Every weld log record traceable to software version and release hash |")
before("## 13. Open items",
       "### 12.1 Safety concept — software view (Proposed, CP-26 as modified by B-18)\n\n"
       "The IPC software is **not** a safety-related part of the control system. No safety function depends on it. "
       "PLr values come from the risk assessment REQ-002 (OI-31); the table is the starting allocation.\n\n"
       "| SF | Safety function | Realised by | Software role |\n"
       "|---|---|---|---|\n"
       "| SF1 | Emergency stop of robot, positioner and emission | E-stop chain → UR safety I/O, drive STO, LightWELD "
       "interlock loop | None (state shown on HMI) |\n"
       "| SF2 | Guard door / Class 4 enclosure open → no emission, safe stop | Door switches → safety relay / TwinSAFE → "
       "LightWELD interlock loop, UR safeguard stop, drive STO | None |\n"
       "| SF3 | Emission only with all permissives (door, clamp contact, gas, wire, drive ready) | PLC (LightWELD "
       "driver) | Requests only (WeldStart / WeldEnd) |\n"
       "| SF4 | IPC failure → emission off, axes stopped | PLC heartbeat watchdog: emission off, then positioner stop "
       "(6.1) | M8 advances the heartbeat counter; M10 forwards it |\n"
       "| SF5 | Burn-through: emission while the welding axis is stationary outside the start dwell (all modes; "
       "B-17) | PLC (hard-wired Robot Moving and drive in-motion signals, emission-duration cap, 6.1) | None (sends "
       "weld_axis, dwell_s, expected_duration_s) |\n"
       "| SF6 | Robot speed, force and zone limits | UR safety configuration (checksum in the context record), "
       "designed by the named safety engineer from the workspace exported by M4 / M7 | Never changes limits "
       "(design rule); planning-relevant limits exported as data to `cell.yaml` with the checksum and checked by M4 / "
       "M7 with a slow-down margin (HW-1, B-25) |\n\n"
       "**SF3–SF5 are process interlocks in the standard PLC until REQ-002 assigns a PLr and a safety-rated "
       "realisation (B-18).** The full split into (A) credited safety functions and (B) process interlocks (laser "
       "key, passive beam containment with OI-24, fume extraction) is made by the named PLC & safety engineer at the "
       "HW-1 gate.\n\n"
       "Rules: software may stop the cell but never overrides or emulates SF1–SF6; the `sim` mode of M10 reproduces "
       "the DES-002 weld-event table (process interlocks), never the safety functions; every change of SF allocation "
       "is a change of REQ-002.\n\n"
       "### 12.2 Error budget — first estimate (Proposed, CP-25 as modified by B-21)\n\n"
       "Target: the combined error at the seam (root sum of squares of independent sources, plus systematic terms "
       "added linearly) must stay inside the allowed seam deviation (OI-16; working value 0.1–0.2 mm, AD5), "
       "separately in the lateral direction (in the seam plane: weld position) and along the beam (with nozzle "
       "contact: contact force or contact loss, OI-22). Values are Fact or Assumption / [TBD] until measured in A54. "
       "The Allocation column is the target for each source; DES-003 targets of A51 / A52 / A53 / A55 are derived "
       "from it (each current target alone exceeds AD5).\n\n"
       "**Robot weld (robot moves, positioner at rest; robot-only and Mode B)**\n\n"
       "| # | Source | Lateral | Along-beam | Allocation | Status | Reduced by |\n"
       "|---|---|---|---|---|---|---|\n"
       "| E1 | UR5e pose repeatability | ±0.03 mm | ±0.03 mm | — | Fact (UR data sheet, ISO 9283) | — |\n"
       "| E2 | UR5e absolute accuracy with nominal kinematics | [TBD] | [TBD] | [TBD] | Fact (bound): nominal "
       "kinematics may be off by centimetres — `ur_calibration` README [R14] | Calibrated kinematics (ADR-18), "
       "measurement in A54 |\n"
       "| E3 | TCP calibration residual (A51) | [TBD] | [TBD] | [TBD] | To be measured | Better TCP method, A56 |\n"
       "| E4 | Fixture frame calibration (A52) | [TBD] | [TBD] | [TBD] | To be measured | Jig reference features |\n"
       "| E5 | Part tolerance and position in the jig | [TBD] | [TBD] | [TBD] | Customer drawing / jig design | "
       "Go / no-go check (A53) |\n"
       "| E6 | Distortion from manual tack welds (CP-19, OI-26) | [TBD] | [TBD] | [TBD] | Unknown | Tack procedure |\n"
       "| E7 | Mode B index angle error δ × r | [TBD] | [TBD] | [TBD] | OI-25 | Unidirectional index approach, "
       "encoder, A55 |\n"
       "| E8 | Path discretisation and time parameterisation | ≤ 0.01 mm | ≤ 0.01 mm | — | Assumption | Pitch "
       "rule (B-03) |\n"
       "| E9 | Thermal drift of robot and fixture | [TBD] | [TBD] | [TBD] | Assumption | Warm-up, re-check |\n"
       "| E11 | Nozzle wear between TCP checks | [TBD] | [TBD] | [TBD] | Assumption | A56 after tip change and at "
       "shift start |\n"
       "| E12 | Shrinkage from earlier seams in the sequence | [TBD] | [TBD] | [TBD] | Unknown | Seam order |\n"
       "| E13 | Wire aim error (filler wire) | [TBD] | — | [TBD] | Assumption | WPS ψ window, gauge setting at tip "
       "change (B-05) |\n\n"
       "**Mode C (robot at rest, positioner turns)**\n\n"
       "| # | Source | Lateral | Along-beam | Allocation | Status | Reduced by |\n"
       "|---|---|---|---|---|---|---|\n"
       "| E1 | UR5e pose repeatability (one static pose) | ±0.03 mm | ±0.03 mm | — | Fact (UR data sheet) | — |\n"
       "| E2 | UR5e absolute accuracy with nominal kinematics | [TBD] | [TBD] | [TBD] | Fact (bound), [R14] | "
       "Calibrated kinematics (ADR-18) |\n"
       "| E3 | TCP calibration residual (A51) | [TBD] | [TBD] | [TBD] | To be measured | A56 |\n"
       "| E5 | Part tolerance and position in the holders | [TBD] | [TBD] | [TBD] | Customer drawing / jig design | "
       "Go / no-go check (A53) |\n"
       "| E6 | Distortion from manual tack welds | [TBD] | [TBD] | [TBD] | Unknown | Tack procedure |\n"
       "| E7 | Positioner angle error δ × seam radius r (0.43° ⇔ 0.1 mm at r = 13.3 mm) | [TBD] | — | [TBD] | "
       "OI-25 | Encoder, backlash, A55 |\n"
       "| E9 | Thermal drift | [TBD] | [TBD] | [TBD] | Assumption | Warm-up |\n"
       "| E10 | Seam runout about the positioner axis (holder concentricity, part straightness; residual of the "
       "Mode C coaxiality tolerance) | [TBD] | [TBD] | [TBD] | Assumption | Holder design, Mode C tolerance (B-03) |\n"
       "| E11 | Nozzle wear between TCP checks | [TBD] | [TBD] | [TBD] | Assumption | A56 |\n"
       "| E13 | Wire aim error (filler wire) | [TBD] | — | [TBD] | Assumption | WPS ψ window (B-05) |\n\n"
       "Decision rule: the values of OI-30 (with OI-16) are due at the Sim-2 exit. The part-location method (fixed "
       "jig only, or a go / no-go touch check) is decided at the HW-1 gate with measured E2. No runtime per-part "
       "correction in Phase 1 (OI-43). Fixed reference features on the jig (pin / cone) are ordered as a mechanical "
       "item (Nghia Nguyen), needed by HW-1. The budget is reviewed at each calibration (A54).\n\n"
       "### 12.3 Test strategy (CP-31 as modified by B-19 / B-24)\n\n"
       "| Level | Content | Runs |\n"
       "|---|---|---|\n"
       "| Unit | Pure logic without ROS (frame math, mm ↔ m adapter, schemas, DP search on synthetic graphs, context "
       "compare) | Every commit (CI) |\n"
       "| Contract | `awm_interfaces` and OpenAPI: generated clients compile; schema validation of every job file; FK "
       "consistency planner vs browser (20 states, 0.01 mm / 0.01°, B-07); runtime dependency rule (B-06) | Every "
       "commit |\n"
       "| Planner regression | Golden part 564082 + analytical test parts (A25) + one 2-solid part: coverage, "
       "continuity, clearance, reproducibility (4.3, 12); scene sanity C1 including contact-zone poses (B-04) | Every "
       "merge request |\n"
       "| Integration (Sim-1) | `sim_mock` launch: plan → program → verify → replay on mock hardware | Every merge "
       "request / nightly |\n"
       "| System (Sim-2) | Same simulation release through M8 / M9 / M10 sim on URSim + mock positioner; "
       "commanded vs actual against the D09 points (B-15) | Before the Sim-2 exit |\n"
       "| Fault injection (Sim-2) | DES-002 scenario set against M10 sim (B-18, B-19): start refused below 100 % "
       "scaling; scaling change in a weld segment → WeldEnd + FAULT; LaserReady timeout → EmissionOff → FAULT, no "
       "retry; EmissionLost mid-seam; Mode C positioner stall; M8 tick stopped while M10 is alive → emission off; door "
       "open; LightWELD / gas / wire faults; mixed mode → NOT_READY; one changed byte in D09 → refused | CI on URSim + "
       "mock positioner |\n"
       "| Hardware-in-the-loop | Conformance run of the DES-002 scenario set and PLC interlock matrix against the real "
       "PLC program (PLC engineer); Sim-2 fault evidence is provisional until it passes; LightWELD disconnected or key "
       "removed; URSim with the designed UR safety configuration | HW-1 |\n"
       "| FAT | Function and role tests, safety validation checklist (WBS 6.3) | Hardware stage |\n\n"
       "Negative tests are required for every rejection rule (unreachable point, collision, contact check, stale "
       "context, wrong release scope, speed scaling < 100 %).\n\n"
       "### 12.4 Operations and observability (CP-31 as modified by B-18 / B-19)\n\n"
       "- Sim stages: software version (Git commit) and image **digest** (not tag) in the context record of D07, D09, "
       "D10; `run_id`, `job_id` and `seam_id` in the logs.\n"
       "- From HW-1: structured logs (JSON lines) from every node and from `awm_api`; health through ROS 2 "
       "`/diagnostics` (links to UR, PLC, positioner drive, `move_group`, disk space); backup of the job repository "
       "and calibration records to the shared drive daily, restore tested once per release.\n"
       "- Runbooks (bring-up, manual recovery after FAULT, calibration, dry run) and a support-bundle CLI (logs, "
       "rosbag2 excerpt, D09 / D10, context record, versions) are agent deliverables; the agent never touches "
       "hardware, the commissioning engineer runs them (B-18, OI-39).\n\n")

# --- 13. open items -----------------------------------------------------------------------------
rep("| OI-5 | Customer Q&A items 1–4 (robot model, sim vs real demo, path source, part positioning) | ADR-01, A53 | "
    "PM / customer |",
    "| OI-5 | Customer Q&A items 1–4 (robot model, sim vs real demo, path source, part positioning); Phase 1 "
    "demonstration agreement with the customer: demo part, simulation / hardware, part-location method, STEP only "
    "(STL = Phase 2), acceptance evidence (B-22) | ADR-01, A53, 1.2 | Tac Huynh (PM / customer) |")
rep("| OI-7 | Seam list and path JSON schemas (first version) | R1 | Software |",
    "| OI-7 | ~~Seam list and path JSON schemas (first version)~~ — closed (B-14): schemas exist in code (v1); v2 frozen "
    "2026-11-04 (B-13) | — | — |")
rep("| OI-8 | ADR-09 HMI toolkit | R2 | Software + PLC |",
    "| OI-8 | ~~ADR-09 HMI toolkit~~ — closed (B-14): decided by ADR-09 (Web UI) | — | — |")
rep("1 axis in Phase 1 (from the CAD) |",
    "1 axis in Phase 1 (from the CAD). Purchase requirement (B-20): closed loop with encoder; cyclic synchronous "
    "position / velocity supported; EtherCAT or CANopen CiA 402; hard-wired in-motion and fault outputs; STO. Decide "
    "with ADR-22 |")
rep("compliant (spring) nozzle or force control; tack beads on the seam |",
    "compliant (spring) nozzle or force control; tack beads on the seam. Acceptance: compliance travel ≥ 2 × the "
    "along-beam error (12.2) (B-04) |")
rep("J2: robot-only first, Mode B only if robot-only coverage < 100 % (decided 2026-10-29) |",
    "J2: robot-only first, Mode B only if robot-only coverage < 100 % (decided 2026-10-29). r_max of the rotating set "
    "sets the Mode C angular step (B-03) |")
rep("execution on hardware (calibration, dry run, trial weld, inspection) assigned before the hardware stage | — | — |",
    "execution on hardware (calibration, dry run, trial weld, inspection) assigned before the hardware stage. "
    "**Exceptions (Proposed, B-18, pending OI-39 / E-03):** the Cell PLC program, DES-002, the alarm catalogue and "
    "the UR safety configuration are owned and authored by the named PLC & safety engineer (OI-4); hardware "
    "execution by a named commissioning engineer + backup; both named by 2026-11-20. The software agent drafts "
    "requirements and runbooks only | — | Linh Tran (amendment) |"
    "\n| OI-29 | TST-002 decision (Go / Pivot / No-Go, ADR-12) on 2026-11-06, chaired by Linh Tran; spike executed by "
    "the software agent 2–5 Nov, checks by Truong Hoang. Missing inputs on 5 Nov → slip ≤ 3 working days, not Pivot. "
    "Outcome table in TST-002 Rev 0.3 §6 (B-01) | ADR-12, Sim-1 | Linh Tran, Truong Hoang |"
    "\n| OI-30 | Error budget values E2…E13 and the allowed seam deviation (with OI-16), due at the Sim-2 exit; the "
    "touch-check decision follows at the HW-1 gate (B-21) | ADR-18, A53, A54 | Welding engineer + software |"
    "\n| OI-31 | Risk assessment REQ-002 with PLr for SF1…SF6 (12.1); split into safety functions and process "
    "interlocks; confirm the PLC rules of 6.1 (emission follows the welding axis, heartbeat) | 6.1, 12.1, PLC | PLC & "
    "safety engineer |"
    "\n| OI-32 | Named human reviewers for the safety-relevant packages, the verification-defining files and the PLC "
    "program (ADR-23) | ADR-23 | Linh Tran (Engineering Manager) |"
    "\n| OI-33 | ~~Launch names `sim` vs `sim_mock` / `sim_ursim`~~ — closed (B-14): rename the launch in PLAN T0.4 "
    "as Rev 0.5 decided | — | — |"
    "\n| OI-34 | ~~Code migration plan Rev 0.2 → Rev 0.5 / 0.6~~ — closed (B-08): migration plan = PLAN.md §2 (keep / "
    "replace per package); T0.1 limited to kept code | — | — |"
    "\n| OI-35 | ~~Sim-1 scope~~ — closed (B-12, B-14): moved to PLAN.md (ordered slice, principle 9) | — | — |"
    "\n| OI-36 | Requalification after a calibration-only change (envelope, re-validated release); Phase 1 rule "
    "meanwhile: automatic full re-plan and re-verify of all jobs (B-25) | ADR-17, A51, A74 | Truong Hoang + welding "
    "engineer |"
    "\n| OI-37 | **Escalation E-01 (board 2026-10-30) — not decided.** Is the TST-002 (ADR-12) decision held on "
    "2026-11-06, chaired by Linh Tran, with the frozen inputs of B-01…B-05, and missing inputs treated as a slip of "
    "at most 3 working days rather than a Pivot? Options: A yes, as proposed; B decide on 2026-11-06 with whatever "
    "evidence exists, missing evidence = Pivot signal; C no fixed date. Board recommendation: A | ADR-12, TST-002, "
    "OI-29 | Linh Tran (Engineering Manager), with Truong Hoang |"
    "\n| OI-38 | ~~Escalation E-02~~ — **closed 2026-11-03: option A (Truong Hoang)**, ADR-17 amended. Amend ADR-17 (decided 2026-10-27) with a `trial` release scope so "
    "that A72 / A73 run through M8 / M9 / M10, and make production a promotion of the same frozen bundle? Options: A "
    "amend now (scope table and CLI refusal before Sim-2, gate code at HW-2); B decide at the HW-1 gate, scope check "
    "built as a data table either way; C keep ADR-17 and run A72 / A73 outside M8 / M9 (breaks AD8). Board "
    "recommendation: A, with B as fallback; C not acceptable | ADR-17, A72, A73, A74 | Truong Hoang |"
    "\n| OI-39 | **Escalation E-03 — not decided.** Amend OI-28 so that a named human owns and authors the Cell PLC "
    "program, DES-002, the alarm catalogue and the UR safety configuration, plus a named commissioning engineer and "
    "a Laser Safety Officer? Options: A amend (PLC & safety engineer and commissioning engineer + backup by "
    "2026-11-20, LSO before HW-2); B amend only for the PLC program; C keep OI-28. Board recommendation: A | OI-28, "
    "ADR-23, 2.3 | Linh Tran, with Truong Hoang |"
    "\n| OI-40 | **Escalation E-04 — not decided.** Phase 1 deliverable on 2026-12-04? Options: A a "
    "simulation-proven system (Sim-2), HW-1 / HW-2 dated in PM-004; B move the Phase 1 end after HW-2; C compress the "
    "hardware stages into December (not recommended). Board recommendation: A, aligned with the customer agreement "
    "(OI-5) | 1.2, 2.3, PM-004 | Tac Huynh (project lead) and Linh Tran |"
    "\n| OI-41 | **Escalation E-05 — not decided.** Do the J1 / J2 WPS use filler wire, and from which side (ψ window "
    "of the TST-002 decision)? Options: A autogenous, free ψ 0–350°; B filler wire leading, ψ window ±15° "
    "(Assumption); C unknown by 2026-11-04: report both cases and decide ADR-12 on the wire-window case. Board "
    "recommendation: answer by 2026-11-04, otherwise C | ADR-12, D05, TST-002 | Truong Hoang (with the welding "
    "engineer and Khang Doan) |"
    "\n| OI-42 | **Escalation E-06 — not decided.** Rebuild the IPC runtime and CI image from the public `ros:jazzy` "
    "base (non-root, pinned digest, micromamba OCC lock) before HW-1, although the environment is fixed to the "
    "demo-based image? Options: A yes, as a HW-1 entry item built in parallel, dev container switched only after "
    "parity tests; B use the demo-based image on the IPC; C switch now, during Sim-1. Board recommendation: A | 8, "
    "8.1, ADR-10 | Truong Hoang |"
    "\n| OI-43 | **Escalation E-07 — not decided.** If the error budget shows the fixed jig cannot meet the band, may "
    "a per-part measured correction re-solve IK at runtime as an exception to design rule 1? Options: A no "
    "exception (reject out-of-tolerance parts or widen the band, OI-16); B bounded exception (same ψ and "
    "configuration label, deterministic re-solve, all checks re-run, bound in `cell.yaml`, logged); C offline "
    "re-plan per part before welding. Board recommendation: decide at the HW-1 gate with measured E2, OI-16, OI-30; "
    "default A; B only with written bounds and ADR-23 review | ADR-18, 12.2 | Truong Hoang |"
    "\n| OI-44 | **Escalation E-08 — not decided.** Which part is demonstrated to the customer in Phase 1 (564082 or "
    "152056 from the customer Q&A, wiki L-009), and is part detection by sensor expected? Options: A 564082 only, "
    "fixed jig, STEP only; B add 152056 as a second job; C part-detection sensor in Phase 1 (own CP and stage). "
    "Board recommendation: A, plus B if its STEP is available by 2026-11-13; C only as a staged CP after HW-1 | OI-5, "
    "1.2, A53 | Tac Huynh |")
closed = ["—", "—", "—"]
add_columns("| # | Item | Needed for | Owner |", ["Needed by (stage)", "Due", "Critical path"], {
    "OI-1": closed, "OI-7": closed, "OI-8": closed, "OI-9": closed, "OI-13": closed, "OI-20": closed,
    "OI-28": ["2.3 (amendment)", "2026-11-20 (OI-39)", "Yes"], "OI-33": closed, "OI-34": closed, "OI-35": closed,
    "OI-2": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-3": ["Sim-2", "Sim-2 start (2026-11-23)", "No"],
    "OI-4": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-5": ["Sim-1", "2026-11-13", "Yes"],
    "OI-6": ["HW-1", "HW-1 entry", "No"],
    "OI-14": ["HW-1 (drive purchase)", "2026-11-20", "Yes"],
    "OI-15": ["HW-1", "HW-1 entry", "No"],
    "OI-16": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-21": ["HW-2", "2026-11-13", "No"],
    "OI-22": ["HW-1", "HW-1 entry", "No"],
    "OI-23": ["HW-1", "HW-1 entry", "No"],
    "OI-24": ["HW-2", "HW-2 entry", "No"],
    "OI-25": ["HW-1", "HW-1 entry", "No"],
    "OI-26": ["HW-2", "HW-2 entry", "No"],
    "OI-27": ["TST-002 decision", "2026-11-06", "Yes"],
    "OI-29": ["Sim-0 exit", "2026-11-06", "Yes"],
    "OI-30": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-31": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-32": ["Sim-2", "2026-11-20", "Yes"],
    "OI-36": ["HW-2", "HW-2 gate", "No"],
    "OI-37": ["Sim-0 exit", "2026-11-06", "Yes"],
    "OI-38": ["Sim-2 (scope table)", "closed 2026-11-03", "Yes"],
    "OI-39": ["Sim-2", "2026-11-20", "Yes"],
    "OI-40": ["Sim-1", "2026-11-13", "Yes"],
    "OI-41": ["TST-002 decision", "2026-11-04", "Yes"],
    "OI-42": ["HW-1", "Sim-2 exit (2026-12-04)", "No"],
    "OI-43": ["HW-1", "HW-1 gate", "No"],
    "OI-44": ["Sim-1", "2026-11-13", "Yes"],
}, default=["[TBD]", "[TBD]", "No"])
before("## Appendix A.",
       "**Dated dependencies outside the OI list (B-22).** `moveit_py` / distance API check: spike day 1. DES-002 "
       "weld-event behaviour table (draft by the software agent, B-18): 2026-11-20. Rotary ground contact for Mode C "
       "and fixed reference features on the jig: mechanical design inputs now (Nghia Nguyen). DES-002 complete and "
       "REQ-002 Rev 0.2: Sim-2 exit. UR safety configuration design and the 'Robot Moving' threshold measured at the "
       "slowest travel speed: HW-1 entry. Laser Safety Officer: HW-2 entry. PLAN.md gets §6 'Risks and triggers' and "
       "PROGRESS.md a weekly dependency check.\n\n")

DST.write_text(s, encoding="utf-8")
print("written", DST)
