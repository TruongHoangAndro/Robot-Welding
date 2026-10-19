# Change proposal for APLSMN601-DES-001 Rev 0.2 → Rev 0.3

Status: proposal by the software agent, for review by Truong Hoang / Linh Tran. `docs/reference/` is not edited.

## CP-1 ADR-10: ROS 2 distribution

**Current text (Rev 0.2):** Ubuntu 22.04 + ROS 2 Humble for Phase 1; migrate to Ubuntu 24.04 + Jazzy in Phase 2.
Reason given: "Humble is the existing, working dev environment (WSL Ubuntu 22.04) and the base of the Tac demo".

**Facts found (2026-10-19, see `env_report.md`):**
- The WSL host has no ROS installed (`/opt/ros` absent).
- The Tac demo runs on **ROS 2 Jazzy** in Docker (`osrf/ros:jazzy-desktop-full`, Ubuntu 24.04, Gazebo Harmonic),
  image `ros2-jazzy-hardware-accelerated:latest`, which already contains MoveIt 2.12, pick_ik, ur_description,
  ur_moveit_config and ros2_control.
- Reusing the demo on Humble would need a backport (Jazzy → Humble), which is the opposite of the stated reason.

**Proposed text:**

| ADR | Decision | Status | Alternatives / reason |
|---|---|---|---|
| ADR-10 | **ROS 2 Jazzy (Ubuntu 24.04) in Docker for development and for the IPC.** Development runs in the dev container `awm_dev` (image derived from the demo image, `docker/Dockerfile` in the workspace) on a WSL2 Ubuntu 22.04 host; the IPC runs the same image. | Proposed — decide at G2 | The demo (DES-001 §10 reuse) is Jazzy; the Jazzy image already has every Phase 1 ROS dependency; Jazzy is supported until May 2029, Humble ends May 2027 (≈ 5 months after Phase 1). Docker gives one reproducible environment for laptop and IPC. |

**Sections to update:** §1.4 (IPC term), §4.1 (L2 runs on "ROS2 Jazzy (Docker)"), §8 deployment
(IPC: Ubuntu 24.04 container on Ubuntu host; Development: WSL2 Ubuntu 22.04 host + `awm_dev` container),
§13 OI-9 (replace "Humble → Jazzy migration" by "IPC Docker runtime: GPU / real-time kernel needs for ur_robot_driver").

**Consequences:** `docs/distro_notes.md` not needed for Humble/Jazzy portability. Real-time behaviour of
ur_robot_driver inside Docker on the IPC must be checked in R2 (new open item).

## CP-2 ADR-02: OCCT subprocess split (`awm_geometry`)

**Current text (Rev 0.2):** B-Rep CAD kernel OCCT (pythonocc-core); M1 `awm_cad` loads STEP with OCCT.

**Facts:** pythonocc-core is available from conda-forge only for its own Python (3.10 env `awm_occ`,
pythonocc-core 8.0.1.2 / OCCT 8.0.1). ROS 2 Jazzy uses the system Python 3.12. Both cannot share one interpreter.

**Proposed addition to ADR-02:** All B-Rep work runs in a pure-Python package `awm_geometry` (no rclpy),
executed in the conda env through a CLI `$AWM_OCC_PYTHON -m awm_geometry.cli <command> --job <dir>`. It reads
and writes job-package files only. ROS-side packages (`awm_cad`, `awm_seam`, `awm_path`) call it through
`subprocess`; the interpreter comes from parameter `occ_python_cmd` (default: env `AWM_OCC_PYTHON`).
The demo used OCC 7.x; API differences to 8.0 are handled inside `awm_geometry` only.

**Sections to update:** §4.2 module catalogue — add row:

| Module | Package | Responsibility | Functions owned | Phase |
|---|---|---|---|---|
| M1/M2 geometry helper | `awm_geometry` | OCCT B-Rep: STEP load/heal, tessellation, edge catalogue, seam fingerprint, edge sampling (points, tangents, face normals) | used by A11–A14, A21, A22, A41, A42 | 1 |

§5.2 job package — add the files written by `awm_geometry` (see CP-3).

## CP-3 Job package additions (§5.2)

Files written by the offline chain that are not yet listed in §5.2:

```
jobs/<PartNo>_<PartRev>/
├── cache/
│   ├── model.brep         D02 cache (healed shape)
│   ├── mesh.stl           display / collision mesh (A13), part frame, metres
│   └── edges.json         B-Rep edge catalogue (ids, type, length, end points, fingerprints) for A21 picking
└── programs/<prog_rev>/verify/report.json   D15
```
`edges.json` is derived data (regenerated from model.step), not a new data item.

## CP-4 Smaller corrections

| # | Where | Issue | Proposal |
|---|---|---|---|
| 1 | §10, §1.3 | F09 (DES-004) is not mentioned anywhere | Add F09 to the reuse table or state it is not relevant |
| 2 | Header | Status "Draft", Reviewed by "[TBD]" but "Approved by" filled | Leave Approved by empty until review |
| 3 | §3–§8 | Figures `APLSMN601-DES-001_figures/*.png` not delivered with the .md | Add the folder next to the document |
| 4 | DES-003 xlsx | Level-1 status shows `#REF!`; stray rows "Paramter collision / Collison list" and "Web UI - Later"; D16 missing in Data Items; D09/D10 format "URScript / ROS2 trajectory" vs JSON in DES-001 | Fix in DES-003 Rev 0.2 |

## CP-5 Partial seams (A22 start / end) in D04

DES-003 lists "start / end" as a seam attribute (A22). Implemented as optional `range_m`
[s_start, s_end] (arc length along the edge chain, weld direction) in `seams.json`, set from start /
end picks. Reason found in testing: a full 360° pipe-to-flange seam is not reachable from a fixed
UR5e base with the 45° torch (the robot-side arc puts the wrist over the base axis); it is welded
as three arcs with their own approach. Proposal: add `range_m` to the D04 description in §5.1 and
note in §2.2 that seam splitting (or a positioner, Phase 2) is the answer to orbital seams.

## CP-6 D07 contains the checked joint path

DES-003 A44 outputs a "joint path"; DES-001 §5.1 describes D07 only as poses. Implemented:
`path.json` keeps the tool poses in the part frame (design rule 3) **and** the joint path that M4
checked, tied to the calibration hash. M6 reuses that joint path after an FK check against the
transformed poses (A61) instead of re-solving IK (IK solvers are stochastic; re-solving could pick
another branch than the verified one). A calibration change makes path.json stale (hash) → M4
re-runs. Proposal: add to §4.3 / §5.1.

## CP-7 Kinematics: two groups on the same chain

`weld_arm` (Cartesian paths, OMPL) uses KDL: deterministic tracking of the 2 mm Cartesian steps.
`weld_arm_search` uses pick_ik (global) only to find IK branches of approach poses. pick_ik for
the Cartesian paths gave 2/3–4/5 success on the pipe arcs; KDL 10/10. Proposal: §4.3 says "MoveIt2
+ pick_ik"; change to "MoveIt2 (pick_ik for IK search, KDL for Cartesian paths)".

## CP-8 Verification resolution

move_group validates time-parameterised waypoints only; a full test run found a torch↔part contact
between waypoints of an OMPL air move. M6 and A71 now check every segment densely (≤ 0.02 rad per
step). Proposal: §6 / §12 Correctness: "dense collision check of the complete program".
