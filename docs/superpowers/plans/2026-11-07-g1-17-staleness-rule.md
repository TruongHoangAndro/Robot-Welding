# G1.17 — One staleness rule for D07 / D09 (close-out A3, audit M3)

Date: 2026-11-07 · Agent: planner · Branch: `g1-17-staleness-rule` (from GotoCode 6de1347)
References: DES-001 Rev 0.5 §4.3 (each step records the hash of its inputs so a later step detects stale inputs),
§5.2 (job package rules: `inputs` with SHA-256, a module refuses inputs whose hash does not match; repository root is a
parameter), design rule 4 (no silent fallbacks); close-out plan `2026-11-07-g1-closeout.md` task A3; Goal 1 audit M3.

Write scope (named by the lead, AGENT_TEAM.md §1 exception): `src/awm_path`, `src/awm_program`, new module
`src/awm_core/awm_core/staleness.py` with tests in `src/awm_core/test`, this plan. Not `awm_api` (web switches
`jobs.document_state` to the shared function in wave B), not `path.schema.json` (no schema change needed).

## Problem (audit M3)

1. D07 records its inputs by paths relative to the job folder. In `golden_repo_g1`, `cell/model/head` is a symlink
   into `.worktrees/spike_repo`; the recorded name is `../../../../spike_repo/cell/model/head/head.yaml`.
   `awm_api.jobs.document_state` skips inputs that resolve outside the repository (→ `path_state: ok`, Generate
   enabled) while the `awm_program` CLI checks them (→ "stale … head.yaml: missing; re-plan"). Two rules, two answers.
2. A D07 planned before the head-record change c01ba5b is not detected: head.yaml's hash did not change, but the
   head record the planner derives from it did.

## Decisions

* **D1 — inputs outside the repository are refused at plan time and stale for every reader.** Input names stay
  relative to the job folder (convention of every job document, `jsonio.make_header(..., job.dir)`; changing only
  D07 to a repository-root base would make it the one job file with another base and every reader would need a
  per-document base), computed from the symlink-resolved path (`jsonio.input_entry` already resolves). New rule: the
  resolved path of every input must lie inside the resolved repository root. `awm_path.cli plan` refuses (exit 1,
  nothing written) when an input resolves outside, naming the input and the fix (replace the link by a copy,
  `cp -rL`). Every checker (shared function below) reports such an input as stale with the same text.
  Why refuse instead of recording it specially: DES-001 §5.2 makes the repository the unit that is backed up,
  released and hashed; an input outside it can change or vanish without the repository knowing, and a skipped check
  is a silent fallback (design rule 4). The normal flow (S10 head import) writes the head into `cell/model/head`, so
  only test / demo repositories that link the head are affected; they get one clear answer from every checker.
  Links that resolve inside the repository stay allowed (recorded by their resolved path).
* **D2 — one function, `awm_core.staleness`.** `document_verdict(job, name, repo_root, doc=None) -> Verdict`
  (`state` ∈ {missing, ok, stale}, ordered `problems`, `message()`), and `document_state(...) -> str` with exactly the
  signature of `awm_api.jobs.document_state`, so the wave-B switch is one line
  (`from awm_core.staleness import document_state` in place of the local definition). Rules, in order, per input:
  outside the repository → stale; not a file → stale (`missing`); hash differs → stale (`changed since this file was
  created`). Names resolve against the job folder only (the API's "document folder" fallback is dropped: every writer
  uses the job folder, and a fallback could accept a different file of the same name). An unreadable document →
  `missing` (as the API).
* **D3 — planner record version.** `awm_core.staleness.PLANNER_RECORD_VERSION = 1`, written by `awm_path.cli plan`
  as `planner.params.record_version` (free-form, hashed into `params_sha256`; no schema change). A D07 (a document
  with a `planner` block) without it or with a lower value → stale, "planned with an older planner; re-plan"; a
  higher value → stale, "planned with a newer planner …". Bump rule (docstring): any change of the planner output or
  of the head record (`HeadGeometry.as_record`, head.yaml format) that changes what a D07 means. A guard test in
  `awm_path` pins the key set of the head / description record to the version so such a change fails until the
  version is bumped.
* **D4 — D09 inherits D07 staleness.** For a program document (`program_rev` present), each JSON input that is a D07
  is checked with the same rule; its problems are reported as `<name>: <problem>`. A program generated from an old or
  stale D07 is stale.
* **D5 — users.** `awm_path.cli plan`: refuses inputs outside the repository (D1), checks `seam_axis.json` with the
  shared rule before planning (same answer as `awm_api.seam_axis_state` after the switch), and new subcommand
  `awm_path.cli check --job J [--path F]` prints `state <ok|stale|missing>` + problems (exit 0 ok / 1 otherwise).
  `awm_program.cli generate`: D07 checked with the shared rule (replaces `jsonio.check_inputs`), schema validation
  unchanged.

## Tasks

- [ ] T1 `awm_core/test/test_staleness.py` (red): unchanged inputs → ok; changed / missing input → stale; input via a
      symlink leaving the repository → stale with "outside the repository"; symlink inside the repository → ok;
      D07 without / older / newer `record_version` → stale with the texts; non-D07 needs no version; D09 with a
      stale D07 input → stale; `document_state` signature = API's; missing / unreadable document → missing.
- [ ] T2 `awm_core/staleness.py` (green).
- [ ] T3 `awm_path` tests (red): plan with a head folder linked out of the repository → refused, nothing written;
      D07 written by plan has `planner.params.record_version == PLANNER_RECORD_VERSION` and the shared verdict is ok;
      `check` subcommand ok / stale; version guard on the head / description record keys.
- [ ] T4 `awm_path/cli.py` (green).
- [ ] T5 `awm_program` tests (red): D07 without `record_version` → refused "planned with an older planner; re-plan";
      symlinked-out head → same verdict from `awm_path.cli check`, `awm_program.cli generate` and
      `awm_core.staleness.document_state`.
- [ ] T6 `awm_program/cli.py` (green).
- [ ] T7 golden fixture (`awm_path/test/test_golden_g13.py`): copy the D07 inputs (placement.json, part.json, head
      folder) instead of linking them out of the temporary repository; run G1.3 + G1.5 golden tests with
      `AWM_SPIKE_REPO` and `AWM_OCC_PYTHON`.
- [ ] T8 package check: `scripts/dx "colcon build --symlink-install --packages-select awm_core awm_path awm_program
      && colcon test --packages-select awm_core awm_path awm_program && colcon test-result --verbose"`.

## Follow-ups for other owners

* web (wave B): replace `awm_api.jobs.document_state` by `from awm_core.staleness import document_state`; API tests
  that relied on skipped outside-repository inputs or the document-folder fallback change to stale.
* lead / C1: `golden_repo_g1` (`.worktrees/demo`) links its head folder out of the repository: replace the link by a
  copy (`cp -rL`) before re-planning, else plan is refused (D1).
