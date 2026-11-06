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
| planner CLI prefix (G1.3) | — | `AWM_PLAN_CMD` | `python3 -m awm_path.cli` (Assumption) |
| built front-end | `--web-dist` | `AWM_WEB_DIST` | `src/awm_web/dist` via `web.sh` |
| bind / port | `--host` / `--port` | `AWM_API_HOST` / `AWM_API_PORT` | `127.0.0.1` / `8000` |

Users file format (never commit a real one; `config/users.example.json` shows the shape):
`{"users": [{"username": "…", "role": "Operator|Engineer|Admin", "password": "pbkdf2_sha256$…"}]}`.
`python3 -m awm_api.users list --file <file>` lists users. Tokens are kept in memory (8 h); a restart logs everyone out.

## Endpoints and roles

| Endpoint | Roles | Notes |
|---|---|---|
| `POST /auth/login` | — | `{token, role, expires_at, username}` |
| `GET /jobs`, `GET /jobs/{id}` | all | detail adds `placement`, `placement_note`, `seam_axis` (`missing` / `ok` / `stale`), `collision_checked` |
| `POST /jobs` (multipart `step`, `part_no`, `part_rev`, `component`, `up_axis`) | Engineer, Admin (S1) | 202 + import run: `awm_geometry.cli import` then `export-view` in the OCC process, into a hidden staging folder, promoted to `jobs/<PartNo>_<PartRev>` only on success. 409 if the job exists or the cell is busy |
| `GET /jobs/{id}/view/{view.glb\|topology.json}` | all | D20 |
| `GET /jobs/{id}/placement` | all | 404 `/problems/no-placement` for a job without `placement.json` (every new upload): Goal 1 uses the golden-part placement from the cell CAD; the API never invents one |
| `GET /jobs/{id}/files/{path}` | all | job-relative files; no `..`, no symlink leaving the job (403) |
| `GET /jobs/{id}/seams` | all | D04 as stored; 404 `/problems/no-seams` before the first save |
| `PUT /jobs/{id}/seams` | **Engineer only** (CP-21) | picks → seam CLI → D04 (+ `seam_axis.json` when `placement.json` exists; header `X-AWM-Seam-Axis`). 422 with field pointers or the CLI message; 409 when the cell is busy |
| `POST /jobs/{id}/plan` (`{seams?: [...]}`) | Engineer | 202 + plan run. 409 `no-seams` / `no-placement` / `seam-axis-missing` / `seam-axis-stale` / `run-busy` |
| `GET /runs/{run_id}` | all | run document of the contract |
| `DELETE /runs/{run_id}` | Engineer, Admin | SIGTERM to the process group (SIGKILL after 10 s); 409 when finished |
| `WS /runs/{run_id}/ws?token=…` | all | pushes the run document on every change (≤ 10 Hz) until it ends |
| `GET /jobs/{id}/simulation` | all | 404 `/problems/no-simulation` until a v2 `path.json` and a placement exist (G1.5 / G1.6) |

## Runs (T2.7)

- Run folder `<repo>/logs/runs/<run_id>/`: `run.json` (served by `GET /runs`), `log.txt` (merged stdout / stderr),
  inputs (`upload/<file>.step`); the planner writes `path.json` there and it is copied to `<job>/path.json` only
  when the run succeeds. Seam saves log to `<repo>/logs/seam_saves/<job>-<time>/` (`picks.json`, `log.txt`).
- One slot per cell for imports, plans and seam saves (409 `/problems/run-busy` with `active_run`).
- Outputs of a failed or cancelled run are never promoted. Runs that were active when the API stopped are marked
  failed at the next start.
- Progress: a CLI may print `progress <0..1> <text>` lines (stdout or stderr); otherwise only step boundaries move it.

## Adapters waiting for other branches (`awm_api/adapters.py`)

| CLI | Owner / branch | Command used now | Status |
|---|---|---|---|
| seam build | seam-process, `g1-2-seam-axis` (G1.2) | `<AWM_SEAM_CMD> build --job <job> --picks <picks.json> [--no-axis]` | taken from the G1.2 CLI help; tested with a fake. Until G1.2 merges, a save answers 500 `/problems/seam-cli-missing` |
| planner | planner, G1.3 | `<AWM_PLAN_CMD> plan --job <job> --collision off --out <run>/path.json [--seams S01 …]` | **Assumption**: module name and `--out` / `--seams`; tested with a fake. Until G1.3 exists a plan run fails with the CLI error |

## Contract deviations (to be folded into the OpenAPI by the lead)

1. `SeamPick` also accepts `positioner_mode` (`sync` default | `C` | `B`), `positioner_angles_deg` (Mode B only, one
   per arc) and `start_offset_mm` (D04 2.1 of G1.2, CP-59); `arc_overlap_mm` defaults to 2 mm (Assumption, no WPS yet).
2. `WS /runs/{run_id}/ws` (progress push requested for G1.4); polling `GET /runs/{id}` still works.
3. `JobDetail` extra fields `placement`, `placement_note`, `seam_axis`, `collision_checked`; login returns `username`.
4. Response header `X-AWM-Seam-Axis` on `PUT /seams`.

## Tests

```bash
scripts/dx "colcon build --symlink-install --packages-up-to awm_api && colcon test --packages-select awm_api && colcon test-result --verbose"
scripts/dx "cd src/awm_api && python3 -m pytest -q test"          # quick loop
```

`test/fake_cli.py` stands in for the OCC, seam and planner CLIs. `test_import_occ.py` uploads a synthetic STEP made by
`awm_geometry make-test-part tube_tube` through the real OCC process (skips only if `AWM_OCC_PYTHON` is unset).
