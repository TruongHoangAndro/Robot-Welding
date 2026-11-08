# awm_api — Web UI backend (Sim-1 / Goal 1 subset)

FastAPI backend of the Web UI (DES-001 ADR-09, §4.5, §7.3, §7.4; PLAN.md T2.7, T3.1 subset, T3.3 subset; master plan
G1.4). Contract: `docs/generated/contracts/awm_api_sim1.openapi.yaml` (deviations listed below).

- Everything under `/api/v1`; every error is an RFC 9457 problem (`application/problem+json`, `type` = `/problems/<name>`).
- **No rclpy, no OCC** in this package (B-11, ADR-15; guarded by `test_boundaries.py`). The engineering CLIs run as
  subprocesses. The browser talks only to this backend; nothing here commands motion, the PLC or the laser.
- Units: mm / deg / s, exactly as the job documents carry them; the backend does not convert anything.
- Goal 1 (CP-58): nothing checks collision. Job details carry `collision_checked: false`; the UI shows
  "kinematic only — collision not checked".

## Start (for Truong)

All commands from the workspace root on the host; they run inside `awm_dev`.

```bash
# once: build the Python package (and after pulling changes)
scripts/dx "colcon build --symlink-install --packages-up-to awm_api"

# once: create the users file (git-ignored location; the password is asked twice and stored as a PBKDF2 hash)
docker exec -it awm_dev bash -lc "source /opt/ros/jazzy/setup.bash && cd $HOME/ros2/aplsmn601_ws && source install/setup.bash && \
  PYTHONPATH=/opt/awm_venv/lib/python3.12/site-packages:\$PYTHONPATH \
  python3 -m awm_api.users add --file <repo>/secrets/awm_api_users.json --user truong --role Engineer"

# start awm_api + the built front-end (builds awm_web on the first start; --rebuild after front-end changes)
scripts/dx "src/awm_api/scripts/web.sh --repo <repo>"
# open http://localhost:8000/   — stop with Ctrl+C
```

`<repo>` is the job repository root (DES-001 §5.2, folders `cell/`, `jobs/`, `logs/`), e.g. a copy of the spike
repository with the golden part. Options of `web.sh`: `--port 8000`, `--host 127.0.0.1` (default: this PC only;
`awm_dev` uses the host network), `--secrets <file>`, `--rebuild`. The API docs are at `/api/v1/docs`.

| Setting | Argument | Environment | Default |
|---|---|---|---|
| repository root | `--repo` | `AWM_REPO_ROOT` | required |
| users file | `--secrets` | `AWM_API_SECRETS` | `<repo>/secrets/awm_api_users.json` |
| OCC interpreter (ADR-15) | — | `AWM_OCC_PYTHON` | set in `awm_dev` |
| seam CLI prefix (G1.2) | — | `AWM_SEAM_CMD` | `python3 -m awm_seam.joint_cli` |
| planner CLI prefix (G1.3) | — | `AWM_PLAN_CMD` | `python3 -m awm_path.cli` |
| program CLI prefix (G1.5) | — | `AWM_PROGRAM_CMD` | `python3 -m awm_program.cli` |
| description render (G1.6) | — | `AWM_DESCRIBE_CMD` | `python3 -m awm_api.describe` |
| replay CLI prefix (G1.20) | — | `AWM_REPLAY_CMD` | `ros2 run awm_bringup replay_program` |
| H-CAD head folder | — | `AWM_HEAD_DIR` | `<repo>/cell/model/head` — **must be inside the repository root** |
| built front-end | `--web-dist` | `AWM_WEB_DIST` | `src/awm_web/dist` via `web.sh` |
| bind / port | `--host` / `--port` | `AWM_API_HOST` / `AWM_API_PORT` | `127.0.0.1` / `8000` |

**Inputs inside the repository, no links (G1.17 staleness rule, `awm_core.staleness`).** The API uses the same rule as
the planner and program CLIs: a job document (path.json, programs/*/program.json, seam_axis.json, placement.json,
cad_check.json) is stale when any recorded input resolves outside `repo_root`, is reached through a symbolic link,
is missing or changed, or (D07) was planned with another planner record version. So `AWM_HEAD_DIR` and every other
input must be a plain folder / file inside `repo_root`; a linked head folder (`ln -s`) or one outside the repository
makes every plan stale and the planner refuses it — copy it instead (`cp -rL`). S2 / S4 show the reasons.

Users file format (never commit a real one; `config/users.example.json` shows the shape):
`{"users": [{"username": "…", "role": "Operator|Engineer|Admin", "password": "pbkdf2_sha256$…"}]}`.
`python3 -m awm_api.users list --file <file>` lists users. Tokens are kept in memory (8 h); a restart logs everyone out.

## Endpoints and roles

| Endpoint | Roles | Notes |
|---|---|---|
| `POST /auth/login` | — | `{token, role, expires_at, username}` |
| `GET /jobs`, `GET /jobs/{id}` | all | detail adds `placement`, `placement_note`, `seam_axis` (`missing` / `ok` / `stale`), `path` + `path_problems`, `program` (`{rev, state, problems}` of the newest revision), `collision_checked` |
| `POST /jobs` (multipart `step`, `part_no`, `part_rev`, `component`, `up_axis`) | Engineer, Admin (S1) | 202 + import run: `awm_geometry.cli import` then `export-view` in the OCC process, into a hidden staging folder, promoted to `jobs/<PartNo>_<PartRev>` only on success. 409 if the job exists or the cell is busy |
| `GET /jobs/{id}/view/{view.glb\|topology.json}` | all | D20 |
| `GET /jobs/{id}/placement` | all | 404 `/problems/no-placement` for a job without `placement.json`: an assembly job (CP-60) gets it from the cross-check (`source: job_cad`); a single-part job needs a manual / copied placement (test helper, Assumption); the API never invents one |
| `GET /jobs/{id}/files/{path}` | all | job-relative files; no `..`, no symlink leaving the job (403) |
| `GET /jobs/{id}/seams` | all | D04 as stored; 404 `/problems/no-seams` before the first save |
| `PUT /jobs/{id}/seams` | **Engineer only** (CP-21) | picks → seam CLI → D04 (+ `seam_axis.json` when `placement.json` exists; header `X-AWM-Seam-Axis`). 422 with field pointers or the CLI message; 409 when the cell is busy |
| `PUT /jobs/{id}/placement` (`{flange_T_part: {position_mm, rpy_deg \| quat_xyzw}}` or `{copy_from}`) | **Engineer** | v2/placement with `source: study`, `verified: false`, Assumption in `created_by`; part mesh = D20 `view.glb`. Makes seam_axis.json / path.json stale (header `X-AWM-Seam-Axis`). 409 `no-view` / `no-placement` (copy source) / `run-busy` |
| `POST /jobs/{id}/plan` (`{seams?: [...], travel_speed_mm_s?: > 0, collision?: off}`) | Engineer | 202 + plan run; the speed goes to `--travel-speed-mm-s` (absent = planner default, Assumption). 409 `no-seams` / `no-placement` / `seam-axis-missing` / `seam-axis-stale` / `run-busy`, the CP-60 / CP-62 gate types (below), `collision-unavailable` for `collision: on` (Goal 2). With a valid CP-62 override the run writes `cad_check_override` into D07 before promotion; the program run copies it into D09 |
| `GET / POST / DELETE /jobs/{id}/check/override` (`{reason}`) | GET all; POST / DELETE **Admin** | CP-62 override of cross-check 4 for kinematic-only planning: `cad_check_override.json` (v2/cad_check_override: by, reason, check-4 findings, SHA-256 of `cad_check.json`). POST 409 `override-not-applicable` (check 4 not red, or another check red), `cad-check-stale`, `run-busy`. Gate on seams / Plan: red passes only with check 4 the only red check, a current check, a valid override for this `cad_check.json` and collision off; else 409 `cad-check-red`, `cad-check-override-void` (check changed after the override), `cad-check-override-invalid`, `cad-check-override-kinematic-only` |
| `POST /jobs/{id}/program` | Engineer | 202 + program run (G1.5 CLI `generate --job`); outputs = `programs/P###/program.json` from the `result ok` line; a `result refused …` reason (e.g. speed above the feasible maximum) is the run problem. 409 `no-path` / `path-stale` / `run-busy` |
| `GET /cell/robot_description` | all | `{urdf, head_model: hcad, unit}`: rendered by `awm_path.description.load_planning_description` in a subprocess (`awm_api.describe`), comments stripped, mesh URIs → `/api/v1/cell/meshes/…`; cached by the frames.yaml / head.yaml hashes. 409 `no-cell-model`, 500 `description-failed` |
| `GET /cell/meshes/head/{path}`, `GET /cell/meshes/package/{pkg}/{path}` | all | read-only mesh files below the head folder / a package share named by the URDF; mesh extensions only; `..`, absolute paths, symlinks leaving the folder → 403 |
| `POST /jobs/{id}/replay` (`{program_rev?}`) | Engineer | S4 "Replay in RViz" (G1.20): 202 + run `replay` of `<AWM_REPLAY_CMD> --program <job>/programs/<rev>/program.json` (newest revision by default) against the sim_mock the user launched with the line in `simulation.replay.launch_command` (`ros2 launch awm_bringup sim_mock.launch.py repo:=<repo root> trajectory_controllers:=robot_positioner`, same container and ROS domain as the API). Only `--program` is passed; the CLI refuses non-mock hardware and sends nothing. `run.result` = the CLI line `replay {json}` (status, `final_deviation_deg`, `max_tracking_error_deg`, segments); a failed run has `problem.cause` `sim-not-running` / `controller-absent` / `not-mock` / `start-state` / `error` / `no-result` with a detail that says what to do. 409 `no-program`, `program-stale` (with `problems`, before any process), `run-busy`; 404 `program-not-found` |
| `GET /cell/scene`, `GET /cell/meshes/scene/{file}` | all | S4 cell view: objects of `cell/model/scene_manifest.json` (parent `world` = static, `positioner_flange` = turns with θ; pose mm at θ = 0) and their STL (manifest-listed files only, else 403). 404 `no-scene` |
| `GET /runs/{run_id}` | all | run document of the contract (+ `result` for runs whose CLI prints a result line) |
| `DELETE /runs/{run_id}` | Engineer, Admin | SIGTERM to the process group (SIGKILL after 10 s); 409 when finished |
| `WS /runs/{run_id}/ws?token=…` | all | pushes the run document on every change (≤ 10 Hz) until it ends |
| `GET /jobs/{id}/simulation` | all | S4 payload: `source` (`program` = newest current D09, else `path`), `program`, `program_rev`, `program_state`, `program_problems`, `path`, `path_state`, `path_problems`, `replay` (launch line, ROS domain, readiness), `placement`, `seams`, `collision_checked`, URLs, `notes`. A program whose inputs changed (re-planned path) is not played. 404 `/problems/no-simulation` without placement or path / program |

## Runs (T2.7)

- Run folder `<repo>/logs/runs/<run_id>/`: `run.json` (served by `GET /runs`), `log.txt` (merged stdout / stderr),
  inputs (`upload/<file>.step`); the planner writes `path.json` there and it is copied to `<job>/path.json` only
  when the run succeeds. Seam saves log to `<repo>/logs/seam_saves/<job>-<time>/` (`picks.json`, `log.txt`).
- One slot per cell for imports, plans, programs, replays and seam saves (409 `/problems/run-busy` with `active_run`).
- Outputs of a failed or cancelled run are never promoted. Runs that were active when the API stopped are marked
  failed at the next start.
- Progress: a CLI may print `progress <0..1> <text>` lines (stdout or stderr); otherwise only step boundaries move it.

## Adapters waiting for other branches (`awm_api/adapters.py`)

| CLI | Owner / branch | Command used now | Status |
|---|---|---|---|
| seam build | seam-process, `g1-2-seam-axis` (G1.2) | `<AWM_SEAM_CMD> build --job <job> --picks <picks.json> [--no-axis]` | taken from the G1.2 CLI help; tested with a fake. Until G1.2 merges, a save answers 500 `/problems/seam-cli-missing` |
| planner | planner, G1.3 (merged) | `<AWM_PLAN_CMD> plan --job <job> --collision off --out <run>/path.json --head-dir <head> [--travel-speed-mm-s V] [--seams S01 …]` | tested with a fake and on the golden copy |
| program | program-verify, G1.5 (not merged) | `<AWM_PROGRAM_CMD> generate --job <job>` | contract of the G1.5 brief; tested with a fake. Until G1.5 merges a program run fails (module not found) and S4 plays the path |

## Contract

`docs/generated/contracts/awm_api_sim1.openapi.yaml` was brought to what the server serves in G1.6 (SeamPick D04 2.1
fields with `range_mm` refused in Goal 1, run WebSocket under `x-awm-websockets`, JobDetail fields, the new endpoints;
`/jobs/{id}/release` is marked `x-awm-implemented: false`). `test_openapi_contract.py` fails when a served route is
missing from the contract or the other way round.

## Tests

```bash
scripts/dx "colcon build --symlink-install --packages-up-to awm_api && colcon test --packages-select awm_api && colcon test-result --verbose"
scripts/dx "cd src/awm_api && python3 -m pytest -q test"          # quick loop
```

`test/fake_cli.py` stands in for the OCC, seam, planner, program and description CLIs. `test_describe_real.py` renders
the real description (synthetic head folder and cell frames of the planner tests) and checks it equals the planner's
URDF apart from the rewritten mesh URIs. `test_import_occ.py` uploads a synthetic STEP made by
`awm_geometry make-test-part tube_tube` through the real OCC process (skips only if `AWM_OCC_PYTHON` is unset).
