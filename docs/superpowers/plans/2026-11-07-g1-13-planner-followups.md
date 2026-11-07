# G1.13 — Planner follow-ups to the cell-sim G1.7 / G1.8 changes

Date: 2026-11-07 · Agent: planner · Branch: `g1-13-planner-followups` (from GotoCode f599d26)
References: DES-001 Rev 0.5 §4.3 (M4), §5.3 (TCP at the nozzle tip), §5.4 (collision model: `nozzle_tip` the only
contact link, conduit capsules), §6.3 Option A1 (one 7-joint controller); CP-56 decision 2 (F9 nozzle from the
description in use, one source); G1.7 (`sim_mock trajectory_controllers:=split|robot_positioner`); G1.8 (conduit at
the head.yaml cable exit, `<awm_conduit source=…/>`, review M6); ADR-14 (units).

## Requests (cell-sim, 2026-11-07)

1. `controller_action` (`awm_path/params.py`, `awm_bringup/config/awm_params.yaml`) names
   `/joint_trajectory_controller/follow_joint_trajectory`, which does not exist with
   `sim_mock trajectory_controllers:=robot_positioner`.
2. The URDF carries `<awm_conduit source="explicit|rear_estimate|parametric"/>`; every planning record must state it.
3. H-CAD with head.yaml `nozzle_len_mm` 0: `nozzle_body` / `nozzle_tip` are empty links, `tcp_laser` at the head.yaml
   TCP. `head_from_urdf` / `contact_params_for` must handle it without a silent fallback.

## Decisions (inside `src/awm_path` only)

* **R1.** No `awm_path` code sends goals to `controller_action`; the field is declared by `params.declare` and read
  by `awm_verify` (Replayer, v1 chain on `sim.launch.py`). Document the field and add a pure helper
  `awm_path.params.check_controller_action(action, action_names)` that raises `ControllerActionMissing` with a message
  naming `trajectory_controllers:=split` when the 7-joint controller runs instead. The move_group backend already
  fails fast through `awm_bringup.scene_sync.check_controller_mode`. Requests: program-verify — call the helper in
  `Replayer.wait_ready`; cell-sim — comment in `awm_params.yaml` next to `controller_action`.
* **R2.** `head_from_urdf` reads exactly one `<awm_conduit source>` (child of `<robot>`), value in
  {explicit, rear_estimate, parametric}; missing, duplicated or unknown → `ValueError` (no default source). New
  `HeadGeometry.conduit_source`; `as_record` adds `conduit_source` and `conduit_source_assumption`
  (true for `parametric` and `rear_estimate`). `gate_g2.head_assumption` states the source (and that the conduit
  size is always an Assumption, TST-002 §3.1); a head record without `conduit_source` is refused.
* **R3.** Empty `nozzle_tip` (no collision, no visual) and, if present, empty `nozzle_body` = no parametric nozzle:
  `HeadGeometry.nozzle_geometry = 'none'`, radius / s_c / tip length `None`; `tool0_T_tcp` is read as before (the
  kinematic planner needs only it). A half-empty nozzle (one link empty, the other with geometry) is refused.
  **F9 refuses** (`contact_params_for` raises `NoNozzleError`, a `ValueError`): there is no disc, and taking a disc
  from the head mesh would be a geometry the description does not state (no silent fallback, CP-56 one source).
  The head assumption text states "no parametric nozzle; F9 not available".

## Tasks

- [x] T1 Plan (this file).
- [x] T2 Tests first: `test_params.py` (helper messages), `test_core_head.py` (conduit tag read / missing / unknown /
  duplicated; empty nozzle links → `nozzle_geometry 'none'`, record, `contact_params_for` refusal; half-empty refused),
  `test_gate_g2_runner.py` (assumption text with conduit source, no-nozzle text, missing source refused),
  `test_description_record.py` (synthetic H-CAD head with explicit cable exit → `explicit`; map variant →
  `rear_estimate`; hbox → `parametric`; synthetic head with `nozzle_len_mm` 0 → `none`, F9 refused).
- [x] T3 Implement in `params.py`, `core/head.py`, `gate_g2.py`.
- [x] T4 `colcon build --packages-select awm_core awm_description awm_bringup awm_path && colcon test
  --packages-select awm_path`.
