# Manual jog page and Teach home — design

Date: 2026-11-13 · Spec for **CP-64** (manual robot jog in the browser + taught cell home), to be added to
`docs/generated/DES-001_change_proposal.md`. Direction decided by Truong on 2026-11-13 (brainstorming). Implements a new
screen **S11** and data item **D22**; builds on DES-001 Rev 0.5 §4.5 (Web UI), §5.3 (frames), §7.3 (roles), §7.4 (API),
ADR-13 (simulate first, driver behind an interface), ADR-14 (mm / deg), CP-61 (teach on the UI, same pattern).

**Status: APPROVED by Truong 2026-11-13.** Frame question #1 decided (readout shows both `tcp_laser` and `tool0`; §3a);
D22 → planner seed / "go home" confirmed as a follow-up (out of this slice). CP-64 added to the change proposal. Next:
plan with `superpowers:writing-plans`, then build. The DES-001 Rev 0.5 copy is edited during implementation per the
repo convention (changes logged in the change proposal).

## 1. Problem / intent

- "Go home" and the planner IK seed use a **nominal** home (SRDF `group_state "home"` in
  `awm_description/srdf/awm_cell_v2.srdf.xacro` + `config/initial_positions_v2.yaml`), marked **Assumption**. There is no
  way for the user to set a home that fits the real cell layout.
- The user wants a teach-pendant / RViz-like way to move the robot by hand, position it sensibly, and **teach home** —
  defined while declaring the cell, the same spirit as teach TCP (CP-61).
- Decision (Truong, 2026-11-13): build it as a **browser teaching tool first** (move the model + IK, no controller),
  architected so the same jog commands route down to a controller (mock / URSim / real) later via ADR-13.

## 2. Scope boundary (mandatory — safety)

- **Phase 1 = simulation only.** S11 moves the **browser URDF model** and computes IK / FK in `awm_api` with the pure
  analytic UR kinematics (no MoveIt, no controller, no robot). It never opens a ROS control graph and never commands any
  hardware. This matches what the Goal-1 demo runs today (web + browser playback; no `controller_manager` / URSim up).
- **Jogging a real or mock/URSim controller is OUT of this scope.** Continuous manual motion next to a person is not
  covered by §7.3 ("Start is a request; physical button required"); it needs its own safety design (enabling device /
  deadman, reduced speed, UR in Remote, hardwired E-stop, REQ-002) before it may drive any controller. Left as an open
  item (OI-new). The architecture keeps the path open (§6), but the controller backend is not enabled here.
- The teaching tool does **not** check collisions (no planning scene is running); it checks **joint limits** and
  **reach** only. Collision checking belongs to the controller-backed jog (later) and to the planner (existing).

## 3. User and flow

**Screen S11 "Manual control (Jog)", role Admin** (home is part of cell declaration, like S7 calibration and S10 cell
model; CP-63 means an Admin has every Engineer right, but S11 edit stays Admin as a cell-setup screen). The cell model
must be saved (S10) so the robot URDF and the positioner joint exist.

Layout (teach-pendant style, beside a 3D view of the saved cell + robot):
1. **Joint jog** — J1…J6 of the UR5e and the **positioner joint** (7th axis): per-joint `−` / `+` buttons, step size
   selector (0.1 / 1 / 5 °), live joint value, limit bar; a value past its soft limit is refused with a message.
2. **Cartesian jog** — `−` / `+` along X / Y / Z and rotation about X / Y / Z, step selector (1 / 5 / 10 mm, 1 / 5 °),
   with a **frame toggle: Tool (default) / Base** (builder's call, 2026-11-13: teach pendants offer both; Tool-frame is
   the common default). Each press asks `awm_api` for the IK of the stepped pose (analytic UR, branch nearest the current
   config) and updates the model; out-of-reach / limit / wrist-singular is reported and the model does not move.
3. **Move-to** — type a target pose (TCP X Y Z + orientation, mm / deg) → IK → the model animates to it. A convenience
   for positioning; **not stored**.
4. **Teach home** — save the **current** configuration (6 robot joints + positioner angle) as the cell home (D22). A
   confirmation notes that the nominal SRDF home is superseded and any consumer of home becomes stale.
5. **Show current home** — read-only view of the stored D22 (joint values, taught_by, taught_at).

Units in the UI are mm / deg (ADR-14). Degrees ↔ radians convert only through the `awm_core` adapter at the URDF / IK
boundary.

### 3a. Jog / readout frame (decided by Truong, 2026-11-13, question #1)

**The live readout shows BOTH poses: `tcp_laser` (nozzle tip) and `tool0` (flange)**, side by side (mm / deg). Cartesian
jog and move-to act at **`tcp_laser`** (what a pendant user expects); the `tool0` readout is informational.
The `tool0 → tcp_laser` offset comes from the **current cell TCP taught in S10** (`head.yaml`, `tool0_T_tcp_mm` +
`beam_axis`), i.e. the nominal CAD TCP — **not** the D08 calibration (the laser `tcp_laser` entry of
`awm_core.calibration` is not populated yet, and measured D08 replaces nominal only on hardware). The browser already
reads this offset from the rendered URDF (`robot.ts` `TCP_LINK = 'tcp_laser'`), so `/cell/fk` and `/cell/ik` compose the
same offset from `head.yaml`; `/cell/fk` returns both `tool0` and `tcp_laser` poses for the readout. Consequence: if the
cell has no TCP yet, the `tcp_laser` readout and the Cartesian tools are disabled and only the `tool0` readout, joint jog
and teach home work. **Teach home stores joint angles only, so it is frame-independent and works regardless.**

## 4. Data — D22 cell home

`cell/model/home.json` (one per cell, written by S11):

```json
{
  "schema_version": "2.0",
  "created_by": "<admin username>",
  "created_at": "<ISO-8601 Z>",
  "source": "taught",
  "inputs": [{ "name": "cell/model/frames.yaml", "sha256": "<hash>" }],
  "robot_joints_deg": { "shoulder_pan_joint": 0.0, "shoulder_lift_joint": -90.0,
                        "elbow_joint": 0.0, "wrist_1_joint": -90.0, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0 },
  "positioner_joint_deg": 0.0
}
```

- Tied to the cell model by the `frames.yaml` hash so re-importing the cell flags home as stale (same staleness pattern
  as roles / axis / TCP). Joint names = the D07 order already used in `awm_web/src/lib/playback.ts`.
- **Supersedes** the nominal SRDF home for consumers that read D22; when D22 is absent the SRDF nominal stays the
  default (no behaviour change for existing jobs). Wiring D22 into the planner IK seed (`ChainParams.home_joints_rad`)
  and the runtime "go home" (Sim-2 state machine, §6.1) is a **follow-up** decision, not part of this slice.

## 5. API (`awm_api`, additions to §7.4; all Admin / `CELL_WRITE` except reads)

| Endpoint | Method | Purpose | Role |
|---|---|---|---|
| `/cell/fk` | POST | joints (deg) → TCP pose (mm / deg), for the live Cartesian readout | CELL_READ |
| `/cell/ik` | POST | target TCP pose + current joints → nearest IK solution (deg) or a reach / limit / singularity problem | CELL_WRITE |
| `/cell/home` | GET | read D22 (or 404 if not taught) + staleness | CELL_READ |
| `/cell/home` | PUT | write D22 from the current joints | CELL_WRITE |

FK / IK are computed with `awm_path.core.kinematics.UrAnalyticKinematics` (wraps `ur_analytic_ik`, already in the venv;
pure Python, no ROS). Its methods are `fk(q)` and `ik(T)` on `base_link_T_tool0` (the flange) — it has **no TCP notion**;
the `tool0 → tcp_laser` offset is composed on top (see §3a). **Verified 2026-11-13:** the running `awm_api` interpreter
(web.sh env) imports `awm_api` and `awm_path.core.kinematics` and constructs `UrAnalyticKinematics` in-process, so no
subprocess bridge is needed (unlike the OCC path). The positioner joint is not part of UR IK: it is passed through
unchanged (Cartesian jog acts on the 6 UR joints; the positioner is joint-jog only).

**Singularity (builder's call, 2026-11-13):** `/cell/ik` reports wrist-singular using the planner's existing check
(`awm_path.core` wrist-2 margin, `min_wrist_singularity_margin_rad`), so S11 and the planner agree.

## 6. Architecture / how it is built (ADR-13)

- Front-end: a new `awm_web` page `JogPage.tsx` reusing the robot URDF loader and `lib/robot.ts`
  (`parseRobot`, `setJoints`, `limitViolations`) and the cell scene of `SimPage` / `CellPage`. Joint jog is local
  (set joints, check limits). Cartesian jog and move-to call `/cell/ik`; the live TCP readout calls `/cell/fk` (or an
  equivalent FK in the browser). Nav entry `#/jog`, Admin only (like `#/cell`).
- Back-end: a small `jog`/`home` module in `awm_api` calling `UrAnalyticKinematics`; D22 read/write through the existing
  job/cell JSON helpers with schema validation.
- **Interface for later (not enabled now):** the jog action is a single front-end call (`/cell/ik` → set model). The
  controller-backed jog (mock / URSim / real) would add a second backend behind the same call that streams the stepped
  target to a jog controller (MoveIt Servo or small `follow_joint_trajectory` goals) — gated by the §2 safety design.

## 7. Goals / non-goals / anti-goals

- **Goals:** jog the UR5e (joint + Cartesian) and the positioner (joint) in the browser; move-to a typed pose; teach and
  store a cell home (D22) with limit / reach feedback; Admin only; mm / deg.
- **Non-goals:** driving any controller (mock / URSim / real); collision checking in the jog page; poses other than home
  (no park / load — decided 2026-11-13); wiring D22 into the planner seed / runtime "go home" (follow-up).
- **Anti-goals (failure even if it "works"):** any path that could command a real or mock controller from this page in
  Phase 1; a home taught in radians or stored in metres (must be deg / mm via the adapter); a jog that moves the model
  past a joint soft limit without refusing.

## 8. Open items

- **OI-new (safety):** controller-backed / real-robot jog — enabling device, reduced speed, Remote mode, E-stop,
  REQ-002. Must be designed before S11 is allowed to drive any controller.
- **OI-new (integration):** make the planner IK seed and the runtime "go home" read D22 when present (follow-up slice).
- **Open:** default step sizes and joint soft-limit source for the positioner (D16 vs the URDF limits) — builder's call,
  defaulting to the URDF limits and the step sets in §3.

## 9. CP-64 entry (proposed, for `DES-001_change_proposal.md` — pending Truong)

> **CP-64 Manual jog page (S11) and taught cell home (D22)** (decided by Truong, 2026-11-13). A new Admin screen S11
> jogs the robot (joint + Cartesian) and the positioner (joint) in the browser (model + analytic IK only, no controller;
> ADR-13 interface kept for a later controller-backed jog behind a safety design) and teaches the cell **home** (6 robot
> joints + positioner angle) into `cell/model/home.json` (D22), superseding the nominal SRDF home for consumers that read
> it. New API `/cell/fk`, `/cell/ik`, `/cell/home` (§7.4). **Supersedes:** the SRDF nominal home as the only home source.
> **Unchanged:** offline/online split (ADR-05) — S11 is a setup tool, not runtime; simulation-only (no controller /
> hardware); units mm / deg (ADR-14). **Impact:** §4.5 (S11), §5.2 (cell/model/home.json), §7.3 (S11 Admin), §7.4 (new
> endpoints), §13 (two open items: real-robot jog safety; D22 → planner seed / "go home").
```

## 10. Next step

After approval: plan with `superpowers:writing-plans` (TDD slices — kinematics endpoints, D22 schema + read/write, the
JogPage, nav / role), then build. If Truong wants the controller-backed jog, that is a separate design behind the §2
safety gate.
