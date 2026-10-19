# awm_verify (M7, A71-A74) and the chain runner

| Command | Function |
|---|---|
| `ros2 run awm_verify verify --job <job> [--rev P001]` | A71: inputs, continuity, joint limits, events at rest, dense collision check, replay on mock hardware -> `verify/report.json` (D15) |
| `ros2 run awm_verify record --template dry_run` / `record --job ... --type dry_run ...` | A72 / A73 records entered by a person (no robot is driven) |
| `ros2 run awm_verify release --job <job> --rev P001 --approved-by NAME --scope simulation\|production` | A74: `release.json` (D10), program directory made read-only |
| `ros2 run awm_verify run_chain --job <job> [--step ... --part-no ... --part-rev ... --picks ...] [--approved-by NAME] [--start-sim]` | A1 -> A7 in one command |

Scope `simulation` is marked as not valid for production; `production` needs PASS dry-run and
trial-weld records of the same program version. Templates: `templates/dry_run.json`,
`templates/trial_weld.json`.
