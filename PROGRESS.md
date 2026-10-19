# PROGRESS

Read this first after a restart. Continue from the first open task. Plan: `PLAN.md`.

| Phase / task | Status | Notes |
|---|---|---|
| Phase -1 environment (awm_dev, scripts, Dockerfile, env report, change proposal) | done | commit 14252f5 |
| Phase 0 discovery (demo map) | done | docs/generated/demo_discovery.md |
| Phase 1 plan | done | PLAN.md |
| T1 awm_core | done | 22 tests (ROS py3.12 + conda py3.10) |
| T2 awm_geometry import/edges/test parts | done | OCC 8 API; explicit mm→m; BRepOffset_Analyse convexity |
| T3 awm_geometry sampling | done | exact curve evaluation; colcon runs OCC suite via conda (12 tests) |
| T4 awm_cad | done | mocked + real OCC import test |
| T5 awm_seam | done | picking, chain, attributes, export, rematch (16 tests) |
| T6 awm_process | done | WPS lookup, SAMPLE gate, process.json (7 tests) |
| T7 awm_interfaces | done | builds; §7.1 v0 |
| T8 awm_description | done | xacro + SRDF + MoveIt cfg, mock hw only (6 tests) |
| T9 awm_bringup | done | sim.launch.py headless, D16 params, repo template, SimLaunch test helper (7 tests) |
| T10 awm_path pure | done | torch axis, tool roll, approach/retract (10 tests) |
| T11 awm_path MoveIt | done | plate 2/2, pipe 2 arcs 2/2, unreachable + TCP-mismatch negative tests (sim) |
| T12 awm_program pure | open | |
| T13 awm_program MoveIt | open | |
| T14 awm_verify | open | |
| T15 end-to-end | open | |
| T16 docs | open | |

## Last error
none (resolved: pick_ik global mode + cost settings, see decision log)

## Decision log
- 2026-10-19: awm_dev runs as uid 1000 (image user `ubuntu`), HOME=/tmp/awm_home; no fix_owner step needed.
- 2026-10-19: demo entrypoint bypassed (forces ROS_DOMAIN_ID=0); demo FastDDS profile unset in scripts/dx.
- 2026-10-19: git identity set repo-local (Truong Hoang) because none was configured.
- 2026-10-19: pick_ik `mode: global`, `minimal_displacement_weight: 0`, `cost_threshold: 0.001`: with local mode or a displacement weight no IK was ever found for poses far from the seed.
- 2026-10-19: full 360 deg pipe seam is NOT reachable from the fixed UR5e with the 45 deg demo torch (reach map: CCW fails -150..-135 deg, CW fails 135..165 deg; tool0 passes over the base axis). Implemented A22 start / end as optional `range_m` partial seams (schema v1 optional field, OI-7) and weld the pipe as two arcs. Moving the fixture does not fix it (x=0.47/0.50 tested).
- 2026-10-19: `tool_roll` parameter (travel | up); default `travel` (x = travel direction). `up` gave worse reach on the pipe.
- 2026-10-19: test part pick on a cylinder seam vertex is refused as ambiguous (by design); pipe pick moved to 90 deg.
- 2026-10-19: awm_dev has no init process: stopped launches leave <defunct> zombies. Harmless; add `--init` when the container is next recreated (docker/README.md).
