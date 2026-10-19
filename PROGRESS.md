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
| T4 awm_cad | open | |
| T5 awm_seam | open | |
| T6 awm_process | open | |
| T7 awm_interfaces | open | |
| T8 awm_description | open | |
| T9 awm_bringup | open | |
| T10 awm_path pure | open | |
| T11 awm_path MoveIt | open | |
| T12 awm_program pure | open | |
| T13 awm_program MoveIt | open | |
| T14 awm_verify | open | |
| T15 end-to-end | open | |
| T16 docs | open | |

## Last error
none

## Decision log
- 2026-10-19: awm_dev runs as uid 1000 (image user `ubuntu`), HOME=/tmp/awm_home; no fix_owner step needed.
- 2026-10-19: demo entrypoint bypassed (forces ROS_DOMAIN_ID=0); demo FastDDS profile unset in scripts/dx.
- 2026-10-19: git identity set repo-local (Truong Hoang) because none was configured.
