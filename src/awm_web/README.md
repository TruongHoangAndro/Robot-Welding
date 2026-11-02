# awm_web — D20 viewer (S2-lite)

Read-only browser viewer of the D20 3D view model (`view.glb` + `topology.json`, contract
`docs/generated/contracts/D20_view_model.md` v2.0). It lets you check the CAD module (M1) output: solids, B-Rep
faces and edges with their ids, and the joints between separate solids. Pulled forward from PLAN.md WP3; no
`awm_api` backend, no seam saving, no robot, no ROS.

Stack (DES-001 ADR-09): Vite + React + TypeScript + three + @react-three/fiber + @react-three/drei. Node and npm
exist only in the `awm_dev` container; run every command through `scripts/dx` from the workspace root.

## Run

```bash
# from the workspace root (host); <dir> holds view.glb and topology.json, absolute path
scripts/dx "cd src/awm_web && npm ci && AWM_VIEW_DIR=<dir> npx vite --host 127.0.0.1 --port 5173"
# open http://localhost:5173
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
- No seam picking / saving (S2 proper, WP3), no robot or cell, no backend; the `/data/` middleware exists only in
  the Vite dev / preview servers and is replaced by `awm_api` `GET /jobs/{id}/view` later.
- `topology.json` is checked for version, unit and the fields the viewer uses, not against the full JSON schema
  (the producer and `awm_core` validate that).
- Edge picking: an edge wins over the face in front of it only within a depth slack of 1 % of the model radius
  (≥ 0.2 mm); on large parts an edge just behind a nearby face may still be picked — zoom in to disambiguate.
- Every edge is drawn as its own fat line; parts with many thousands of edges may be slow.

## Development

```bash
scripts/dx "cd src/awm_web && npm run typecheck"   # tsc --noEmit
scripts/dx "cd src/awm_web && npm test"            # vitest
scripts/dx "cd src/awm_web && npm run build"       # dist/ (git-ignored)
scripts/dx "cd src/awm_web && npm run fixture"     # regenerate fixtures/two_tubes (deterministic)
```

`fixtures/two_tubes/` is a synthetic, contract-conforming D20 model made by `scripts/make_fixture.mjs`: two coaxial
tubes Ø 26.6 mm (wall 2.3 mm, 40 mm each) butt-jointed at z = 40 mm, Z up, one closed joint JE1. It is not derived
from a STEP file; its ids are numbered by hand.
