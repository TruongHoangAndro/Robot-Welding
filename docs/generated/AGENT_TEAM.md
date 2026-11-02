# APLSMN601 — Software agent team

Status: accepted by Truong Hoang on 2026-11-02. Prepared by the software agent (lead session).
Plan: `PLAN.md` Rev 0.6 (work packages WP-S, WP0…WP4). Progress: `PROGRESS.md`.

The software is built by one **lead** (the main Claude Code session, working with Truong) and specialist sub-agents
defined in `.claude/agents/`. Truong gives direction, decides escalations and runs the acceptance tests in simulation
(DES-003 owners, 2026-10-29). People still own everything on real hardware (DES-001 OI-39 / escalation E-03).

## 1. Roster

| Agent | Writes (only) | Functions (DES-003) | Plan tasks | ROS_DOMAIN_ID |
|---|---|---|---|---|
| lead (main session) | `awm_interfaces`, `awm_core` schemas and contracts, `PLAN.md`, `PROGRESS.md`, change proposal, `docs/generated/**` | — | T0.1 (contracts part), T0.5, merges after Truong's choice (CLAUDE.md, 2026-11-02) | 42 |
| `planner` | `awm_path` (M4 core library, backends, CLI) | A41–A49 | WP-S S.2–S.7, WP1 | 43 |
| `cad-geometry` | `awm_geometry`, `awm_cad` | A11–A15, A17, A18 | T0.2, T2.1 | 44 |
| `cell-sim` | `awm_description`, `awm_bringup`, `awm_positioner` (mock), `docker/`, `scripts/` | D14, D16 | T0.3, T0.4, T0.6, T0.7 | 45 |
| `seam-process` | `awm_seam`, `awm_process` | A21–A25, A31–A34 | T2.2, T2.3 | 46 |
| `program-verify` | `awm_program`, `awm_verify`, `awm_motion` | A61–A64, A71–A74 | T2.4–T2.6, T4.1, T4.2 | 47 |
| `web` | `awm_api`, `awm_web` | A13 / A21 / A71 UI, A88 | T2.7, WP3 | 48 |
| `runtime` (from 23 Nov) | `awm_cell`, `awm_executor`, `awm_plc_bridge` (sim mode only) | A81–A84 (sim) | T4.3–T4.6 | 49 |
| `reviewer` | nothing (read-only) | — | review of every branch before merge | — |
| `rules-auditor` | nothing (read-only) | — | design / safety rule scan of every branch before merge | — |

A file outside an agent's write scope is changed only by its owner. If an agent needs such a change (for example a new
field in a schema or a new apt package), it stops and lists the request in its report; the lead routes it.
Exception: a task plan in `docs/superpowers/plans/` may name extra files for one task.

## 2. Rules for every agent

Every agent reads `CLAUDE.md` first; its rules win. In addition:

1. **Source of truth.** DES-001 Rev 0.5 (`docs/generated/release_rev0.5/`), the Rev 0.6 draft and the decisions B-01…B-25
   where `PLAN.md` applies them, TST-002 Rev 0.3 draft, DES-003 Rev 0.5. If code and design disagree: stop and report,
   never change the design silently.
2. **Contracts are frozen.** `awm_interfaces`, JSON schemas and the example files of T0.5 change only through the lead.
   Work against the example files, not against another agent's unfinished code.
3. **Tests first.** A task is done only when the test named in `PLAN.md` (or in the task plan) has been run and passes.
   Run it through `scripts/dx` of your own checkout. Report the exact command and its result. Never claim a test you did
   not run.
4. **Units** only through `awm_core.units` (ADR-14). **No part data in source** (seam ids, component names, poses).
   **No partial paths**: a seam is 100 % or rejected with failing points.
5. **Safety.** Mock hardware and URSim only. No real IP address. No code that commands laser emission outside the PLC
   `sim` mode of `awm_plc_bridge`. No raised speed / acceleration limits.
6. **Environment.** Nothing installed on the host. apt / pip only inside `awm_dev` and only by `cell-sim` (or the task
   that names it), mirrored in `docker/Dockerfile`. Never touch `ros2_jazzy_dev_env`, `supabase_*`, `docs/reference/**`,
   `docs/LLM_Wiki/**`.
7. **Git.** Work only on your branch, small commits with the package prefix (`awm_path: …`), Co-Authored-By line as in
   the repository history. Never push, never merge, never rewrite history. `PROGRESS.md` is written only by the lead.
8. **Assumptions** are named and marked "Assumption" in code docstrings and in the report.

## 3. Workflow

```
lead: contract / task brief ─► agent (own worktree + branch) ─► tests green
      ─► rules-auditor + reviewer (parallel, read-only, get the diff, not the agent's conclusion)
      ─► agent fixes findings ─► lead presents the result to Truong ─► merge into GotoCode only on Truong's choice
      ─► lead updates PROGRESS.md
      ─► at a stage exit: Truong runs the acceptance script in simulation
```

- **Worktrees.** `scripts/wt new <agent> <branch>` creates `.worktrees/<agent>` inside the workspace (so `awm_dev` sees
  it through the bind mount) with the agent's `ROS_DOMAIN_ID` in `.awm_domain`. Build and test with
  `.worktrees/<agent>/scripts/dx "<cmd>"`. `.worktrees/COLCON_IGNORE` keeps the main build clean. A single agent working
  alone may use the main checkout when its task plan says so.
- **Parallelism.** At most 3 writing agents at the same time (token cost, container CPU for colcon and MoveIt).
- **Branch names.** `wp<n>-<task>-<short>` (for example `wp2-t21-d20-export`); spike tasks `ws-s<n>-<short>`.

### Report (last message of every writing agent)

```
## Report — <agent> — <task ids>
Branch / commits: <branch>, <short hashes + subjects>
Done: <what works now>
Tests: <command> → <result>   (one line per command actually run)
Not done / skipped: <item — reason>
Assumptions introduced: <item — where marked>
Stop items (code vs design, missing input, out-of-scope change needed): <item>
Requests to other owners: <owner — change>
Wiki updates needed: <item>
```

## 4. Waves

| Wave | Dates | Agents | Exit |
|---|---|---|---|
| 0 | 2–6 Nov | planner (S.2 → S.7, main checkout per task plans); lead: contract freeze T0.5 (Wed 4 Nov) | TST-002 decision Fri 6 Nov (Go / Pivot / No-Go / not decided) |
| 1a | 9–13 Nov | planner (WP1), cad-geometry (T0.2, T2.1), cell-sim (T0.3, T0.4, T0.6) | J1 Mode C by CLI, Fri 13 Nov |
| 1b | 16–20 Nov | planner, seam-process + program-verify, web | Sim-1 demo in the browser, Fri 20 Nov |
| 2 | 23 Nov – 4 Dec | runtime, program-verify, cell-sim | Sim-2 on URSim + mock positioner, Fri 4 Dec |
| HW | after Sim-2 | people (PLC & safety engineer, commissioning) with agent support | HW-1 / HW-2 gates, DES-001 §2.3 |

The wave plan follows the spike outcome of `PLAN.md` §4 (Go / Pivot / No-Go).

## 5. When an agent must stop and report

- code and DES-001 / PLAN disagree, or a change to a frozen contract is needed;
- a required input is missing (PLAN §5, e.g. I2 nozzle tip, I5 filler wire);
- an install or build fails where the task plan says "stop" (e.g. `ur-analytic-ik`);
- a test can pass only by weakening a criterion, a padding, a clearance or a limit;
- anything would touch real hardware, a real network address or laser emission.
