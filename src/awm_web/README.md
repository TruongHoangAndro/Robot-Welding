# awm_web — Web UI front-end

Browser front-end of the Workbench (DES-001 ADR-09, §4.5). Two parts:

1. **Workbench (G1.4)** behind a login, served by `awm_api`: **S1-min** job list and "create job from STEP"
   (upload with progress, then the import run with progress), and **S2** seam picker (below).
2. **D20 file viewer (S2-lite)**: the read-only viewer of `view.glb` + `topology.json` without a backend
   (`#/viewer`, or any URL with `?data=`; section "D20 file viewer" below).

The browser talks only to `awm_api` (same origin, `/api/v1`); never to ROS, the robot or the PLC (DES-001 §7.3).
It sends ids and attributes; display geometry is never used as a weld path. All values mm / degrees.

Stack (DES-001 ADR-09): Vite + React + TypeScript + three + @react-three/fiber + @react-three/drei. Node and npm
exist only in the `awm_dev` container; run every command through `scripts/dx` from the workspace root.

## Workbench: start

```bash
scripts/dx "colcon build --symlink-install --packages-up-to awm_api"   # once
scripts/dx "src/awm_api/scripts/web.sh --repo <repo>"                   # builds dist/ on the first start
# open http://localhost:8000/   (users file and options: src/awm_api/README.md)
```

Development with hot reload: start `awm_api` as above on port 8000, then
`scripts/dx "cd src/awm_web && npx vite --host 127.0.0.1 --port 5173"` and open http://localhost:5173/ — the Vite
dev server forwards `/api` (REST and WebSocket) to `AWM_API_URL` (default `http://127.0.0.1:8000`).

## Workbench: screens

- **Login** — user / password from the `awm_api` users file; the role (Operator / Engineer / Admin) decides what is
  enabled. The server checks every request again.
- **S1-min `#/jobs`** — jobs with part no., rev, status (`importing`, `draft`, `planned`, …). Engineer / Admin:
  "Create job from STEP" (file, part no., rev, optional assembly component, up axis of the file); upload progress,
  then the import run (A11–A14 + D20 export in the CAD process) with progress, log tail and Cancel. Operator: read only.
- **S2 `#/jobs/<id>`** — the job's D20 model; orange = joint edges. Click an edge: its joint is selected (both edges
  highlighted); a non-joint edge gets a note (Goal 1 welds joints between solids only). "New seam on JE<n>" adds a
  seam; per seam: id (`S01`…), sequence, joint type, direction (reversed), start offset mm (closed seams), arcs
  (1–12, equal lengths from the start, CP-53), arc overlap mm, positioner mode `sync` (default, CP-59) | `C` | `B`,
  and for `B` one positioner angle per arc. "Save seams" sends the picks; M2 builds `seams.json` (and
  `seam_axis.json` when the job has a placement). Saved seams are drawn green with the seam frame **Z** (blue arrow,
  out of the material) and the **beam axis −Z** (dashed magenta) every 90° of the seam (quarters of the length; open
  seams also at the end), X at the start and yellow arc start markers. Job panel: placement (a new upload has none —
  Goal 1 uses the golden-part placement from the cell CAD), `seam_axis.json` state. "Plan all seams" starts a plan
  run (progress, Cancel). The banner **"kinematic only — collision not checked"** is on the page header and the
  plan panel (CP-58). Seam editing and planning are Engineer only (CP-21); other roles see the page read only.

## Workbench: S2 additions (G1.6)

- **Placement** (Job panel, Engineer): a job made from a STEP upload has no `placement.json`. Enter the pose of the
  part frame in `positioner_flange` (θ = 0; mm, roll-pitch-yaw in degrees, URDF convention) or copy the placement of
  another job. Saved as `source: study`, not verified — an Assumption for simulation only. Afterwards
  `seam_axis.json` is stale: save the seams again, then plan.
- **Plan**: optional travel speed in mm/s (relative to the part, CP-59 follow-up 2026-11-06); empty = planner default
  (Assumption, recorded in D07). Link "S4 simulation →" once a path exists.

## Workbench: S4-lite simulation `#/jobs/<id>/sim` (G1.6)

Display only (DES-001 §7.3): the browser plays stored joint values; it never talks to ROS, the robot or the PLC.

- Robot from `GET /cell/robot_description` (the planner's URDF, head model `hcad`) through **urdf-loader**; meshes
  from the awm_api mesh endpoints with the bearer token. The positioner joint turns with θ; the part (`view.glb`,
  part frame) hangs on `positioner_flange` through `placement.json` `flange_T_part`.
- Playback source: the newest current program (D09 2.1 `robot_positioner` segments or 2.0 `robot` / `positioner`
  segments with `hold_deg`); before a program exists, the timed weld segments of D07 (no air moves between arcs —
  the view jumps there). Joint values = the stored values at the stored times, linear between consecutive points
  (`lib/playback.ts`); urdf-loader limit clamping is off, values beyond a URDF limit are listed as a warning.
- Seam points coloured by D07 status (ok / ik / limit / singularity / wrist3 / step; failing points of rejected arcs
  located on the D04 samples), TCP trail (FK of every played point; orange = weld), scrubber, play / pause, rate,
  current segment kind / seam / mode, θ, q, cycle time (program) or path duration.
- Banner **"Kinematic only — collision not checked"** whenever the played documents do not say
  `collision_checked: true` (CP-58).
- Travel speed per arc: requested vs `max_travel_speed_mm_s` / `speed_within_limits` of D07; over the limit it says
  "max feasible X mm/s — re-plan with a lower travel speed". Re-plan with a new speed and "Generate program" (G1.5
  CLI through awm_api) are on the page for the Engineer.
- Focus: `part` (centre of the part bounding box on the flange) or `cell`.
- Units: the scene unit is the metre of the URDF; every mm → scene and deg → rad conversion is in `src/lib/units.ts`.
- Not shown yet: the static cell (`cell_view.glb` has no agreed place in the job repository and needs the role map to
  hide the robot / part copies inside it), positioner holders.

## D20 file viewer

```bash
# from the workspace root (host); <dir> holds view.glb and topology.json, absolute path
scripts/dx "cd src/awm_web && npm ci && AWM_VIEW_DIR=<dir> npx vite --host 127.0.0.1 --port 5173"
# open http://localhost:5173/?data=/data/   (or http://localhost:5173/#/viewer)
```

Examples for `<dir>`:

- the committed test fixture: `$HOME/ros2/aplsmn601_ws/src/awm_web/fixtures/two_tubes`
- a CAD export: `<job>/cache/view` written by
  `$AWM_OCC_PYTHON -m awm_geometry.cli export-view --step <file> [--component <name>] [--up-axis +Y] --out <dir>`

Stop the server with Ctrl+C. `npm ci` is only needed once (and after `package-lock.json` changes).

Data loading:

- The dev and preview servers serve the directory `$AWM_VIEW_DIR` read-only at `/data/` (GET / HEAD only, files
  inside that directory only; `..`, encoded `..` and symlinks leaving it get 403).
- The page loads `<base>view.glb` and `<base>topology.json`; `base` is the URL query `?data=` (default `/data/`),
  e.g. `http://localhost:5173/?data=/data/`. Only same-origin relative paths are accepted; an absolute URL
  (scheme or `//host`) is refused with a message.
- Fallback: drag and drop both files onto the page, or use "Open files".
- The viewer refuses files with `schema_version` ≠ "2.0" or `unit` ≠ "mm" (topology.json and `asset.extras` of
  view.glb) and says why. Under "Warnings" it lists ids present in only one of the two files, duplicate ids in
  either file, and GLB node extras (`solid_id`, `joint_edge_id`) that differ from topology.json.

## What it shows

- Faces shaded and coloured per solid (B1, B2, … legend in the side panel); edges as lines; joint edges
  (non-null `joint_edge_id`) in orange and thicker.
- The source `up_axis` is rotated to +Z up for display only. All values stay mm in the part frame; nothing is
  rescaled.
- Orbit (left drag), pan (right drag), zoom (wheel); "Fit to view"; axis gizmo (display axes); a grid on the floor
  of the model with the cell / section size in mm shown bottom left.
- Hover highlights edges (6 px screen tolerance) and faces. Click an edge: edge id, solid, curve type, length,
  closed, adjacent faces, G1 neighbours, joint edge id, fingerprint; for a joint edge also the joint (both edges,
  both solids), and both edges are highlighted. Click a face: face id, solid, surface type, area. Ids in the panel
  are clickable. Esc clears the selection.
- Joints list: id, solid pair, length, closed; click to select and focus the camera on the joint.
- "Show preview frames": the 4 preview frames of each joint: X red arrow, Z blue arrow (out of the material),
  dashed blue line = beam axis −Z. **Preview only — not a weld path** (D04 / M4 own seam frames).
- Status bar: counts of solids / faces / edges / joints, source file / component / up axis, `created_at`,
  `created_by`, data origin, hovered id.

## Limits

- Display only. The browser picks ids; display geometry is never used as a weld path (contract rule 1).
- The file viewer has no seam picking (that is S2 in the Workbench), no robot or cell, no backend; the `/data/`
  middleware exists only in the Vite dev / preview servers (the Workbench loads D20 from `awm_api`).
- G1.6 added S4 playback with robot and positioner (above); the static cell is not shown yet. (G1.4 note: no headless browser
  in `awm_dev`).
- `topology.json` is checked for version, unit and the fields the viewer uses, not against the full JSON schema
  (the producer and `awm_core` validate that).
- Edge picking: an edge wins over the face in front of it only within a depth slack of 1 % of the model radius
  (≥ 0.2 mm); on large parts an edge just behind a nearby face may still be picked — zoom in to disambiguate.
- Every edge is drawn as its own fat line; parts with many thousands of edges may be slow.

## Development

```bash
scripts/dx "cd src/awm_web && npm run typecheck"   # tsc --noEmit
scripts/dx "cd src/awm_web && npm test"            # vitest (lib: seams, api, route, D20 loading)
scripts/dx "cd src/awm_web && npm run build"       # dist/ (git-ignored)
scripts/dx "cd src/awm_web && npm run fixture"     # regenerate fixtures/two_tubes (deterministic)
```

S4 tests: `tests/playback.test.ts` (timeline values = program values at the program times, linear between; 2.1, 2.0
and D07 sources), `tests/fk.test.ts` (jsdom: q + θ played through the URDF chain with urdf-loader reproduce the D07
TCP poses in the part frame within 0.01 mm / 0.01°). The FK test uses `fixtures/s4_synthetic/` (regenerate inside
awm_dev with `scripts/dx "python3 src/awm_web/scripts/make_s4_fixture.py"`: synthetic cell, H-BOX head, poses from the
planner's own FK) and, when `AWM_S4_GOLDEN_DIR` names a folder with `robot_description.json`, `path.json` and
`placement.json` of a real job, also that job (skipped otherwise).

Scripted browser run (Playwright, headless Chromium inside awm_dev; browser folder git-ignored):

```bash
scripts/dx "cd src/awm_web && PLAYWRIGHT_BROWSERS_PATH=$PWD/tmp/pw-browsers npx playwright install chromium-headless-shell"   # once
scripts/dx "cd src/awm_web && AWM_UI_URL=http://127.0.0.1:8048 AWM_UI_USER=<engineer> AWM_UI_PASSWORD=<pw> AWM_JOB=<job> \
  AWM_SHOT_DIR=<folder> [AWM_PLAN=1 AWM_TRAVEL_SPEED_MM_S=3] PLAYWRIGHT_BROWSERS_PATH=$PWD/tmp/pw-browsers node scripts/s4_browser_run.mjs"
```

`fixtures/two_tubes/` is a synthetic, contract-conforming D20 model made by `scripts/make_fixture.mjs`: two coaxial
tubes Ø 26.6 mm (wall 2.3 mm, 40 mm each) butt-jointed at z = 40 mm, Z up, one closed joint JE1. It is not derived
from a STEP file; its ids are numbered by hand.
