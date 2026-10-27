# PROJECT.md — APLSMN601 Automatic Weldments · Knowledge Wiki Schema

| Field | Value |
|---|---|
| Project | APLSMN601 Automated Welding Machine ("Automatic Weldments") — UR5e + IPG LightWELD 1000 robotic laser welding cell, STEP → seam → path → ROS 2 |
| File | `PROJECT.md` (schema layer of the project wiki). Saved for review as `APLSMN601_PROJECT_v0.2.md`; named `PROJECT.md` at the wiki root `{WS}/docs/LLM_Wiki/` |
| Rev | 0.2 — Approved for use by Truong, 2026-10-27 |
| Date | 2026-10-27 |
| Owner | Truong Hoang (curates sources, approves changes) |
| Maintainer | Claude (writes and maintains the wiki, research, code) |
| Method | LLM Wiki pattern (A. Karpathy, `llm-wiki.md` idea file): raw sources → wiki → schema; operations Ingest / Query / Lint |
| Basis | First full read of the project on 2026-10-27 (see Appendix A) |

**Revision history**

| Rev | Date | Change |
|---|---|---|
| 0.1 | 2026-10-27 | First version: three layers, conventions, Ingest / Query / Lint, seed findings L-001…L-017. Wiki planned as a separate Git repo |
| 0.2 | 2026-10-27 | Wiki moved into the ROS 2 workspace repo at `{WS}/docs/LLM_Wiki/` so code and knowledge are pushed together (repo used only by Truong). New Section 4.1 (read-only for coding sessions, `wiki:` commits, `COLCON_IGNORE`, private remote); Sections 0, 3.1, 4, 8.4, 8.5, 12, Appendix B updated |

---

## 0. How to use this file

- **Claude Code (VS Code / WSL):** for wiki work open Claude Code in `{WS}/docs/LLM_Wiki/`. It then loads the workspace `CLAUDE.md` (environment and safety rules) and the wiki `CLAUDE.md` (`@PROJECT.md`). For wiki files this file governs; the safety rules of the workspace `CLAUDE.md` always apply. First command of every wiki session: read `PROJECT.md`, `wiki/overview.md`, `wiki/index.md` and the last 10 entries of `wiki/log.md`. For code work open `{WS}` as before (Section 4.1).
- **Cowork / claude.ai Project:** attach or upload `PROJECT.md`, `wiki/overview.md`, `wiki/index.md`; Claude reads pages from the wiki folder on demand.
- **Commands Truong can type** (Vietnamese or English is fine):

| Command | Meaning |
|---|---|
| `ingest <path / link / pasted text>` | Read a new or changed source and integrate it (Section 8.1) |
| `query <question>` / any technical question | Answer from the wiki, cite pages, offer to file the answer (Section 8.2) |
| `lint [scope]` | Health check of the wiki and the project documents (Section 8.3) |
| `file it` | Save the last useful answer as an insight page |
| `status` | One-screen summary from `overview.md` + open tasks + last log entries |
| `auto` / `step` | Switch ingest between batch mode and one-source-at-a-time with approval (default `step`) |

---

## 1. Purpose and scope

The wiki is a persistent, compounding knowledge base for APLSMN601. Claude compiles each source once, links it to what is already known, records contradictions and keeps everything current. Truong supplies sources and direction and decides; Claude does the bookkeeping, research, analysis and code.

**Focus domains** (every page is tagged with at least one):

| Domain | Covers |
|---|---|
| **WELD** — laser / MIG welding | IPG LightWELD 1000 (Phase 1, PLC-controlled), MIG (Phase 2, process abstraction kept), WPS / presets, wobble, wire feed, gas, weld sequence, Class 4 laser safety, weld quality (macro-etch, standard OI-12) |
| **COLL** — robot collision avoidance and AI | P2 method (ψ × 8 IK branches → ladder-graph DP, ADR-12), collision scene and ACM (DES-001 §5.4), TST-002 spike, Tesseract fallback, "AI Solution" agent concept |
| **UR5E** — UR5e robot functions | Kinematics, 8 analytic IK solutions, singularities, wrist-3 / fiber wrap, ur_robot_driver, speed scaling, URSim, External Control URCap, positioner / 7th axis (ADR-11) |
| **SYNC** — WSL and data synchronisation | Host WSL Ubuntu 22.04, Docker `awm_dev` (ROS 2 Jazzy), conda `awm_occ`, workspace ↔ Git ↔ Google Drive (master / working folder) ↔ `docs/reference` copies ↔ wiki |
| **PIPE** — software pipeline (supporting) | CAD import (OCCT), seam picking, path planning, program, verification, job package D01–D20, frames and calibration, Web UI, PLC bridge |
| **PM** — project management (supporting) | Charter, WBS, schedule, gates G0–G5, Sim-0/1/2, risks, team, document register |

**What the wiki is not:** it does not replace released project documents. Numbered documents (`APLSMN601-<area>-<nnn>`) in the Apillis folders stay the formal source of truth; the wiki explains, links and checks them.

---

## 2. Roles and approval gates

| Who | Does | Does not |
|---|---|---|
| Truong | Adds sources, sets direction, answers questions, approves gates | Need to write wiki pages |
| Claude | Reads sources, writes and maintains all wiki pages, proposes decisions and document changes, researches, writes code in the Git workspace | Edit raw sources, released documents, PLC / PCB / drawing files, or the master folder without explicit approval |

Claude asks for confirmation before each major step (Truong's standing rule): before writing an ingest, before applying lint fixes, before any change to a raw source, before creating or moving folders. Exception: `auto` mode for ingest (Section 8.1).

---

## 3. Architecture — three layers

### 3.1 Raw sources (read-only for Claude)

Raw sources are not copied into the wiki. They are referenced through root aliases so the wiki stays portable:

| Alias | Location | Content | Write rule |
|---|---|---|---|
| `{MASTER}` | `G:\Shared drives\API-US-ENGINEERING\4. PRODUCTS\AUTOMATED WELDING MACHINE\` | Company master, Apillis folder structure 1 CUSTOMER … 12 DOCUMENTS, 99 Archived | Never for tests; release copies only, with approval |
| `{WORK}` | `G:\My Drive\20_Customer\Automatic Welding\` | Working copy for test and edit | New Rev files only, with approval |
| `{WS}` | WSL `$HOME/ros2/aplsmn601_ws` (Git) | ROS 2 workspace, its own `CLAUDE.md`, `docs/reference/` frozen document copies; also hosts this wiki | Code changes per the workspace `CLAUDE.md`; `docs/reference/**` never edited |
| `{WIKI}` | `{WS}/docs/LLM_Wiki/` | This wiki (not a raw source) | Changed only by wiki operations (Section 4.1) |
| `{GDOC}` | Google Slides / Sheets / Docs (read through the Drive connector) | Collision Solution, LASER WELDING FOR AUTOMATION WELDING SYSTEM, Question List, Design Review, Design Review Summary, Automatic Weldments Logs, BoM, UR5e_Welding_Robot_Project_Tracker | Read only |
| `{INBOX}` | `raw/inbox/` inside the wiki | New notes, chat pastes, ideas, meeting notes, code snippets, saved web pages | Truong drops files; Claude moves them to `raw/inbox/_ingested/` after ingest (with approval) |
| `{WEB}` | Manufacturer sites, standards, papers | Datasheets, manuals | Saved to `{INBOX}/web/` with URL and retrieval date before ingest |

Rules:

1. Raw sources are immutable for Claude. If a source is wrong or outdated, Claude writes a change proposal (a task + the proposed text) and, after approval, saves a **new revision file** next to it (README Rules 1–2: `_Rev<x.y>` or `_vX` / `_YYYYMMDD`). Never overwrite, delete, move or rename without asking.
2. PLC projects, PCB files and drawings: read and propose only.
3. Google files: read through the connector; record file ID and modified time on the source page.

### 3.2 The wiki (owned by Claude)

Markdown pages written and maintained by Claude: source summaries, topic and entity pages, decisions, registers, tasks, backlog, insights, lint reports. Truong reads; Claude writes.

### 3.3 The schema (this file)

`PROJECT.md` defines structure, conventions and workflows. Truong and Claude change it together: every change bumps the Rev in the header and gets a `schema` entry in `log.md`.

---

## 4. Folder layout

```
{WS}/docs/LLM_Wiki/                   WIKI_ROOT — inside the ROS 2 workspace repo, pushed with the code (Rev 0.2);
                                      weekly zip backup to {WORK}\13 WIKI\_backup\
├── PROJECT.md                        this schema
├── CLAUDE.md                         one line: @PROJECT.md   (loaded together with the workspace CLAUDE.md)
├── COLCON_IGNORE                     empty marker: colcon build skips this folder
├── .gitignore                        wiki-only ignores (zip backups, Obsidian cache)
├── raw/
│   ├── _manifest.md                  root aliases + list of every raw source (path, rev, mtime, ingested?)
│   ├── inbox/                        new material from Truong, waiting for ingest
│   │   ├── web/                      saved web pages / PDFs with URL + date
│   │   └── _ingested/                processed inbox files (moved after approval)
│   └── assets/                       images and screenshots used by wiki pages
└── wiki/
    ├── overview.md                   one-page project state: stage, focus, next milestone, top risks, top open items
    ├── index.md                      catalogue of every page (Section 9.1)
    ├── log.md                        append-only history (Section 9.2)
    ├── glossary.md                   terms and abbreviations (ψ, ACM, CTWD, ADS, TCP, WPS …)
    ├── _templates/                   page templates of Section 5.3 (source, topic, decision, insight, lint)
    ├── sources/                      one page per raw source (src-*)
    ├── topics/                       concept, entity and module pages (top-*, ent-*, mod-*)
    ├── decisions/                    one page per decision (dec-*), incl. mirrors of DES-001 ADRs
    ├── registers/                    ID registers: documents, functions, data-items, modules, open-items, risks, findings, parameters
    ├── todo-tasks/                   open.md (active tasks), done-YYYY-MM.md
    ├── backlog/                      Phase 2 items, R&D ideas, "later" items (bkl-*)
    ├── insights/                     filed answers, analyses, comparisons, calculations (ins-*)
    └── lint/                         lint reports LINT-YYYYMMDD.md + open-issues.md
```

### 4.1 Sharing the repo with the code (Rev 0.2)

The wiki lives in the ROS 2 workspace repo so that code, document copies and knowledge are versioned and pushed together. The repo is a test workspace used only by Truong.

1. **Read-only for coding sessions.** In a coding session (Claude Code opened at `{WS}`, workspace `CLAUDE.md`, coding prompts) `docs/LLM_Wiki/**` is read-only. Wiki pages change only in a wiki session (Claude Code opened at `{WS}/docs/LLM_Wiki/`) through Ingest, Query (filing) and Lint, with Truong's approval (Section 2).
2. **Separate commits.** Wiki commits start with `wiki:` followed by the log heading, e.g. `wiki: [2026-10-28] ingest | DES-001 Rev 0.4`. Code commits keep the package prefix, e.g. `awm_path: …`. One commit never mixes wiki and code. Filter: `git log --oneline --grep "^wiki:"`.
3. **End of a code session:** code commit first, then one `wiki:` commit (module page, tasks, log entry naming the code commit hash), made in a wiki session.
4. **Build isolation:** `COLCON_IGNORE` keeps colcon out of the folder; nothing in the wiki is a ROS package.
5. **Reference copies:** `{WS}/docs/reference/` stays as the workspace defines it (never edited; git-ignored if the workspace `.gitignore` says so). Wiki pages cite it as `{WS}/docs/reference/<file>`; lint L10 checks the Rev of each copy.
6. **Remote:** push only to a private / company Git remote — the wiki holds summaries of customer drawings and project data (Section 11).
7. **Docker:** the folder is visible inside `awm_dev` through the workspace bind mount; nothing in the container writes to it.

---

## 5. Page conventions

### 5.1 File names

- English, lower case, kebab-case, ASCII only, type prefix: `src-des-001-software-architecture.md`, `top-collision-p2-ladder-graph.md`, `ent-ipg-lightweld-1000.md`, `mod-awm-path.md`, `dec-adr-12-process-motion-planner.md`, `ins-2026-10-28-nozzle-contact-vs-tcp.md`, `bkl-positioner-7-axis.md`.
- One page per **document**, not per revision: the source page carries the current Rev and a revision table. A superseded Rev is noted on the same page.
- Renaming a page requires updating every inbound link in the same operation (Claude does this) and a log entry.

### 5.2 Frontmatter (YAML, every page)

```yaml
---
id: top-collision-p2-ladder-graph        # = file name without .md
title: Collision avoidance — P2 (ψ × IK-branch ladder graph)
type: topic          # source | topic | entity | module | decision | register | task | backlog | insight | lint | overview
domains: [COLL, UR5E]
tags: [collision/p2, ur5e/kinematics, sim-1]
status: current      # draft | current | superseded | open | closed | blocked
ids: [ADR-12, A44, A45, D19, OI-15, F07, C1, C2, C3]   # project IDs this page covers
sources: [src-des-001-software-architecture, src-tst-002-collision-spike]
updated: 2026-10-27
confidence: medium   # high | medium | low — how well the sources support the page
---
```

Source pages add: `doc_no`, `rev`, `raw` (alias path), `raw_mtime`, `ingested`, and for Google files `gfile_id`.

### 5.3 Page bodies (templates)

| Type | Sections, in this order |
|---|---|
| source | Summary (≤ 5 lines) · Key facts (each with § / page ref) · Decisions stated · Open items raised · Conflicts with other sources · Pages updated · Revision history seen |
| topic / entity / module | Summary · Current state (date) · How it works · Parameters (table: name, value, unit, F/I/A, source) · Decisions · Open items · Risks · Related pages |
| decision | ID / ADR ref · Status (proposed / decided / confirmed / superseded / rejected) · Date and decided by · Context · Decision · Alternatives · Consequences · Sources · Supersedes / superseded by |
| register | One table; first column = ID; last columns = source, status, updated |
| task (rows in `todo-tasks/open.md`) | T-### · Task · Owner · Deadline · Status · Source / link. Tasks for the technician get a step checklist (short sentences) and note the 06:30–14:30 Mon–Fri window |
| backlog | Idea · Why · Trigger to revisit · Related IDs |
| insight | Question · Answer (conclusion first) · Method · Evidence · Confidence · Follow-ups |
| lint | Scope · Findings table (L-###, severity, check, pages / sources, finding, proposed fix, status) · Summary |

### 5.4 Links and citations

- Internal links: Obsidian wikilinks `[[top-collision-p2-ladder-graph]]` (readable in Obsidian graph view and as plain text in Claude Code).
- Citation of a raw source: `[[src-des-001-software-architecture]] §4.3` or `({MASTER}\…\file.md §4.3)`. Spreadsheets: `sheet!row`. PDFs: page number.
- Every new page gets at least one inbound link (from `index.md` plus a related topic page).

### 5.5 Evidence markers (Truong's rule 8)

| Marker | Meaning | Required with it |
|---|---|---|
| **[F]** Fact | Stated in a source | Citation |
| **[I]** Interpretation | Claude's reasoning from facts | Short reasoning |
| **[A]** Assumption | Needs confirmation | Owner or OI-# and "confirm by" |
| **[TBD]** | Unknown | Owner if known |

- Prices, stock, lead time, EOL status: always with check date and source, e.g. `(checked 2026-10-27, DigiKey)`.
- Never invent part numbers, register addresses, pinouts or signal names. Unknown → [TBD] + OI.

### 5.6 Units, numbers, language, dates

- **Units: mm, deg, s** for all project data (ADR-14); kg, N, W, bar. Third-party ROS values in m / rad are written as such and labelled. Imperial vendor values keep the original and add SI in brackets, e.g. `12–230 ipm (5.1–97.4 mm/s)`.
- Language: wiki pages in **English**, technical terms unchanged; Claude's chat summaries follow Truong's language (usually Vietnamese).
- Dates ISO `YYYY-MM-DD`, time zone Asia/Bangkok.

---

## 6. Tag taxonomy

| Group | Tags |
|---|---|
| WELD | `weld/laser`, `weld/mig`, `weld/wps`, `weld/sequence`, `weld/quality` |
| COLL | `collision/p2`, `collision/scene`, `collision/ai`, `collision/tesseract` |
| UR5E | `ur5e/kinematics`, `ur5e/driver`, `ur5e/ursim`, `ur5e/positioner`, `ur5e/safety-config` |
| SYNC | `devenv/wsl`, `devenv/docker`, `devenv/conda`, `sync/drive`, `sync/git`, `sync/reference-docs` |
| PIPE | `cad`, `seam`, `path`, `program`, `verify`, `calib`, `plc`, `ui`, `data` |
| SAFETY | `safety/laser`, `safety/robot`, `safety/electrical` |
| PM | `pm/schedule`, `pm/risk`, `pm/docs`, `pm/team`, `pm/customer` |
| Stage | `sim-0`, `sim-1`, `sim-2`, `hw`, `phase-2` |

A page may have several tags; domain codes (WELD, COLL, UR5E, SYNC, PIPE, PM) go in `domains:`.

---

## 7. Project IDs and their registers

| Prefix | Meaning | Canonical source (current) | Wiki register |
|---|---|---|---|
| A11…A95 | Functions (IDEF0) | DES-003 Rev 0.4 | `registers/functions.md` (incl. Sim stage column) |
| D01…D20 | Data items | DES-003 Rev 0.4, DES-001 §5.1 | `registers/data-items.md` |
| M0…M12, `awm_*` | Software modules / packages | DES-001 §4.2, workspace `CLAUDE.md` | `registers/modules.md` + `mod-*` pages |
| ADR-01…15 | Architecture decisions | DES-001 Rev 0.4 §9 | `decisions/dec-adr-*` |
| OI-1…21 | Open items | DES-001 §13 | `registers/open-items.md` |
| F01…F15 | As-built findings, Tac demo | DES-004 Rev 0.1 | `registers/findings.md` |
| C1…C8 | Spike criteria | TST-002 Rev 0.2 | `src-tst-002-collision-spike` |
| WBS x.y | Work packages | PM-003 Rev 0.3 | link to source page |
| G0…G5, Sim-0/1/2 | Gates and simulation milestones | Charter, PM-004 v2 | `overview.md` |
| S1…S9, UI-0…7 | Web screens, UI build steps | DES-001 §4.5 | `top-web-ui` |
| Q&A-1…6 | Question List items | Question List (Google Sheet) | `registers/open-items.md` |
| T-, B-, L-, INS-, DEC- | Wiki-own: tasks, backlog, lint findings, insights, non-ADR decisions | this wiki | own folders |

**Ambiguous IDs must be qualified.** The project already reuses short IDs: releases R1–R5 vs Tracker risks R1–R9; IDEF0 A1–A9 vs Tracker assumptions A1–A6 vs positioner options A1 / A2 (DES-001 §6.3) vs the 4-block "A0 Overview Phase 1" sheet. Write `Rel-R2`, `Trk-R3`, `Trk-A1`, `Pos-A1`, `Ovw-A1` when the context is not obvious; plain `A##` always means the IDEF0 function.

---

## 8. Operations

### 8.1 Ingest

Trigger: `ingest <source>`, a file in `raw/inbox/`, or a new Rev detected by lint.

1. **Locate and fingerprint** — path (alias form), doc no., Rev, mtime, size; check `raw/_manifest.md` whether it was ingested before. A re-ingest works on the difference to the previous Rev.
2. **Read fully** with the method for its type (table below).
3. **Report and plan (Gate 1)** — to Truong, short, in Vietnamese: 5-line summary, key facts, decisions, conflicts found, list of pages to create / update, questions. Wait for OK (skipped in `auto` mode).
4. **Write** — source page; update topic / entity / module / decision / register pages (one source typically touches 5–15 pages); add tasks and backlog items; put each contradiction in `lint/open-issues.md` and in the "Conflicts" section of the pages involved. Never resolve a contradiction silently.
5. **Book-keep** — update `raw/_manifest.md`, `wiki/index.md`, `wiki/overview.md` (if state changed), append to `wiki/log.md`.
6. **Report done** — pages touched, open conflicts, suggested next source.

| Source type | How Claude reads it | Special rules |
|---|---|---|
| Working documents `.md` (DES, TST, PM, REQ) | Full read | Record Rev; map every project ID used to its register |
| Registers `.xlsx` (DES-003, DES-004, REQ-002, schedule, WBS) | Export each sheet to Markdown (openpyxl, values only) | Register rows go to `registers/`; topics only link to them |
| Released `.docx / .pptx` | Text extract; compare with the working `.md` of the same doc no. | If they differ, flag; do not merge |
| Google Slides / Sheets / Docs | Drive connector | Record file ID and modified time |
| PDF datasheets / manuals (IPG, UR, Beckhoff) | Text with page numbers; view pages with figures, tables, pinouts | Facts only with page ref; vendor docs rank above distributor sheets above forums |
| Code (Git, zip) | Tree, package.xml / CMake, launch, config, tests; map files → A-IDs and F-findings | One `mod-*` page per package; link commit hash; no large code copies |
| STEP files | Metadata via OCCT script: units, solids, bounding box (mm), face / edge counts | No geometry in the wiki; link to the part entity page |
| Meeting recordings (mp4) | Needs a transcript (Meet / Drive captions) — Claude cannot listen to audio | Minutes page → decisions + tasks |
| Ideas / chat from Truong | First saved verbatim to `raw/inbox/YYYY-MM-DD_<topic>.md` | Marked "stated by Truong"; ideas go to backlog or topics |
| Web research | Saved with URL + retrieval date | Standards and vendor docs preferred; forums = reference only |

### 8.2 Query

1. Read `index.md` → `overview.md` → relevant pages; check `log.md` for recent changes.
2. Answer **conclusion first**, cite wiki pages and raw sections, mark F / I / A. If a cited source is newer than its ingest date, say so and propose a re-ingest.
3. If the wiki does not cover it: say so, read the raw source directly, then propose an ingest. Project-specific facts never come from general knowledge.
4. **Offer to file**: a reusable answer (comparison, design analysis, calculation, decision prep, explanation) is proposed as `insights/ins-YYYY-MM-DD-<slug>.md`; on OK, write it, link it from the related topics and `index.md`, log it.
5. Output formats on request: Markdown page, comparison table, Mermaid diagram, Marp slides, CSV, Python calculation, interactive HTML explainer (like `APLSMN601_P2_Demo_v2.html`).

### 8.3 Lint

**When:** Claude proposes a lint (does not run it unasked) — weekly at the first session of the week; before each milestone or gate (Sim-0 2026-10-30, Sim-1 2026-11-20, Sim-2 2026-12-04, G2); after every 5 ingests; and whenever a new Rev of DES-001, DES-003, TST-002, PM-003 or the workspace `CLAUDE.md` appears.

| Check | What Claude looks for |
|---|---|
| L1 Revision drift | References to non-current revisions (document registers, §References tables, `CLAUDE.md` "Read first", Claude Code prompts) |
| L2 Decision drift | Superseded decisions still stated as current: MIG in Phase 1, automatic seam detection, Humble, Qt GUI, units in m, runtime path planning |
| L3 Units | Project data not in mm / deg / s; imperial values without SI; unlabelled m / rad |
| L4 Parameter consistency | Seam step (1–3 mm, spike 2 mm), ψ step 10°, padding 3 mm, clearance ≥ 5 mm (C2) / ≥ 10 mm (C4), approach 30–50 mm, wrist-3 limit, fiber bend radius ≥ 50 mm, payload 5 kg, head 1.3 kg, accuracy band 0.1–0.2 mm, wobble range |
| L5 ID integrity | Every A / D / ADR / OI / F / C ID used exists in its register; deleted IDs (A23) not used; ambiguous IDs qualified (Section 7) |
| L6 Laser and robot safety consistency | PLC is the only owner of emission (ADR-04); nothing commands emission outside PLC `sim` mode; enclosure and interlocks validated before first emission; standards cited (ANSI Z136.1, IEC 60825-1/-4, ISO 10218, ISO 12100, ISO 13849-1) |
| L7 Structure | Orphan pages, concepts mentioned ≥ 3 times without a page, source pages not linked from any topic, missing cross-links |
| L8 Staleness | Sources newer than their ingest; tasks and open items past deadline; [A] assumptions older than 30 days |
| L9 Evidence | Claims without source, numbers without unit, prices / lead times without check date |
| L10 Sync | Rev in `{MASTER}` vs `{WORK}` vs `{WS}/docs/reference/`; master README vs working README; duplicate folders (Housekeeping list) |
| L11 Gaps | Missing sources and questions to ask (e.g. IPG automation manual, IPG head CAD, PolyScope version), with suggested owner |

Output: `lint/LINT-YYYYMMDD.md` + update of `lint/open-issues.md` + chat summary. Severity: **High** = safety or wrong design input; **Medium** = drift that can mislead; **Low** = cosmetic. Fixes inside the wiki are applied after OK; fixes to raw documents are only proposed (task + proposed text).

### 8.4 Research, development and code

- Code lives in Git (`{WS}`, README Rule 4) and follows the workspace `CLAUDE.md` (Docker-only ROS, mm adapter, safety rules). The wiki holds no full code; `mod-awm-*` pages record purpose, A-IDs, status, tests, last commit, known issues.
- Every code session ends with a `code` log entry, an update of the module page and of `todo-tasks/open.md`, committed as a separate `wiki:` commit after the code commit (Section 4.1).
- A design change found while coding becomes a proposal in `decisions/` and in `{WS}/docs/generated/DES-001_change_proposal.md`; it is never applied silently to the design.
- Spikes and R&D (TST-002, AI collision agent, positioner sync) produce insight and decision pages; results feed ADR status.
- Safety (same as workspace `CLAUDE.md`): never connect to a real robot, PLC, LightWELD or laser; mock hardware / URSim only; no real IP addresses; software never bypasses safety functions.

### 8.5 Synchronisation (SYNC)

"WSL synchronization" = keeping documents and data consistent across five locations (confirmed by Truong 2026-10-27):

| Location | Holds | Direction of flow |
|---|---|---|
| `{WORK}` Drive working folder | Working `.md` + new Revs | Edit here first |
| `{MASTER}` Drive master folder | Reviewed / released files | `{WORK}` → `{MASTER}` after review, with approval |
| `{WS}` WSL Git workspace | Code; `docs/reference/` copies of DES / TST docs | Drive Rev → `docs/reference/` only when Truong approves a new Rev for coding |
| Docker `awm_dev` | Build and run only (bind mount of `{WS}`) | No files of record inside the container |
| Wiki (`{WS}/docs/LLM_Wiki/`) | Compiled knowledge, same Git repo as the code | Reads all of the above; writes nothing to them without approval; pushed with the code |

Rules: `registers/documents.md` lists each document with its Rev in `{WORK}`, `{MASTER}` and `{WS}/docs/reference/`; lint L10 compares them. [I] If Google Drive for desktop exposes G: inside WSL as `/mnt/g`, it is fine for reading documents but not for building code or for a Git repository (slow and conflict-prone) — to be checked on the dev PC.

---

## 9. index.md and log.md

### 9.1 `wiki/index.md` (content catalogue)

Grouped by section: Overview · Sources · WELD · COLL · UR5E · SYNC · PIPE · PM · Decisions · Registers · Insights · Backlog · Lint. One line per page:

```
- [[top-collision-p2-ladder-graph]] — ψ × 8 IK-branch sampling + DP ladder graph; status Proposed (ADR-12) · updated 2026-10-27 · 3 sources
```

Claude reads the index first for every query and updates it on every ingest. Up to roughly 150 pages the index is enough; beyond that Claude proposes a search tool (grep script or a local Markdown search engine).

### 9.2 `wiki/log.md` (append-only history)

Each entry starts with a fixed prefix so it can be filtered with `grep "^## \[" wiki/log.md | tail -10`:

```
## [2026-10-27] ingest | APLSMN601-DES-001 Rev 0.4 — Software Architecture
- Created: src-des-001-software-architecture, dec-adr-12-process-motion-planner, …
- Updated: registers/open-items (OI-1…21), top-collision-p2-ladder-graph
- Conflicts: L-002 (stale references §1.3), L-004 (pythonocc version)
- Next: ingest DES-003 Rev 0.4
```

Entry types: `ingest` · `query` · `lint` · `decision` · `code` · `sync` · `task` · `schema`.

---

## 10. Source-of-truth order and conflict handling

1. Approved / released document (newest Rev) > working `.md` of the same doc no. > other project files (slides, sheets) > notes and chat > wiki text.
2. Among decisions: the most recent dated decision wins (e.g. decisions of 2026-10-22 supersede the charter's MIG-first plan), but the older document keeps its text until a CR / new Rev updates it — the wiki marks it "superseded by [[dec-…]]".
3. Vendor manual > distributor data sheet > internal team note > forum (Truong's rule 8).
4. A contradiction is never resolved silently: it is recorded (page "Conflicts" section + `lint/open-issues.md`) with both sources and a proposed resolution and owner.

---

## 11. Safety, confidentiality and file rules

- Customer drawings, STEP files, prices, purchase costs and quotations are confidential. The wiki stays on the company Drive / Git; no customer data in public pages; nothing is published, shared, emailed or ordered without Truong's confirmation.
- No delete, move or rename of any file without asking. New versions are new files (`_vX`, `_YYYYMMDD` or `_Rev<x.y>`).
- PLC projects, PCB and drawing files: read and propose only.
- Safety content (laser Class 4, robot) is written conservatively: hardware safety functions are independent of software; any proposal that touches emission, speed limits or safety I/O is flagged High and goes to Truong.

---

## 12. Backup and portability

- The wiki is plain Markdown with relative links and root aliases → it can be copied, zipped or put in Git as one folder.
- **Backup:** the wiki is committed and pushed with the workspace repo (one `wiki:` commit per operation, message = log heading). After each weekly lint, also zip `docs/LLM_Wiki/` as `APLSMN601_Wiki_YYYYMMDD.zip` into `{WORK}\13 WIKI\_backup\` (copy outside Git; Drive keeps versions).
- **Claude Code:** wiki work → open `{WS}/docs/LLM_Wiki/`; code work → open `{WS}`. The workspace `CLAUDE.md` gets `@docs/LLM_Wiki/wiki/overview.md` in "Read first" and the read-only rule of Section 4.1 (patch applied by Truong).
- **claude.ai Project / Cowork:** keep only `PROJECT.md`, `overview.md`, `index.md` in the Project docs (small, stable); the full wiki stays in the folder.
- **Hand-over to a colleague:** send the zip inside the company Drive; Obsidian (free) opens it with graph view.

---

## 13. Initial build plan (each step needs Truong's OK)

| Step | Content | Output |
|---|---|---|
| 1 | Create skeleton at the agreed `WIKI_ROOT` | Folders, `PROJECT.md`, `CLAUDE.md`, empty `index.md`, `log.md`, `overview.md`, `glossary.md`, `raw/_manifest.md` |
| 2 | Ingest batch 1 — core design | DES-001 Rev 0.4, DES-003 Rev 0.4, TST-002 Rev 0.2, workspace `CLAUDE.md` (2026-10-25), README (working, 2026-10-22) |
| 3 | Ingest batch 2 — project management | Charter Rev 0.2, Kickoff Rev 0.1, WBS Rev 0.3, PM-004 v2 + schedule xlsx, REQ-002 Rev 0.1 |
| 4 | Ingest batch 3 — welding and collision | LASER WELDING slides, IPG LightWELD + LightWELD Cobot brochures (PDF), Collision Solution slides, P2 demo v2 |
| 5 | Ingest batch 4 — code | DES-004 + Tac demo zips (`ros2_nvidia_build-UR5E_WorkingDemo` first), `Claude outputs` files, Claude Code prompts v1 / v2 |
| 6 | Ingest batch 5 — customer and parts | Question List, customer drawings, STEP metadata of the 8 models, 2026-10-13 meeting (transcript needed) |
| 7 | First lint | `LINT-20261027.md`, seeded with Appendix A |

Planned first topic / entity pages:

| Domain | Pages |
|---|---|
| WELD | `ent-ipg-lightweld-1000`, `top-weld-sequence-plc-handshake`, `top-wps-and-presets`, `top-laser-safety-class4`, `bkl-mig-process-phase-2` |
| COLL | `top-collision-p2-ladder-graph`, `top-collision-scene-and-acm`, `top-tst-002-spike-status`, `top-collision-ai-agent-concept`, `bkl-tesseract-fallback` |
| UR5E | `ent-ur5e`, `top-ur5e-ik-8-solutions-and-singularities`, `top-ur-driver-speed-scaling-ursim`, `bkl-positioner-7-axis` |
| SYNC | `top-dev-environment-wsl-docker-conda`, `top-sync-drive-git-workspace` |
| PIPE | `top-cad-step-import-occt`, `top-seam-picking`, `top-path-planning-m4`, `top-job-package-data-model`, `top-frames-and-calibration`, `top-web-ui`, `top-sim-stages`, `top-tac-demo-as-built`, `ent-golden-parts` |
| PM | `overview`, `top-team-and-roles`, `registers/documents` |

---

## Appendix A — Findings from the first full read (seed for LINT-20261027)

Read on 2026-10-27 from `{MASTER}` and `{GDOC}`. Status: open, nothing changed in any source.

| L-# | Sev. | Check | Finding | Sources | Proposed fix |
|---|---|---|---|---|---|
| L-001 | High | L4 / L6 | LightWELD fires only when the **nozzle touches the part** (part-contact circuit; robot must press the nozzle on the part). TST-002 §3.1 assumes TCP 10 mm beyond the nozzle tip, C2 requires nozzle clearance ≥ 5 mm and DES-001 ACM lets only the TCP link touch seam faces | LASER WELDING slides 6–7; TST-002 §3.1, §5; DES-001 §5.4; OI-13 | Resolve OI-13 before the TST-002 result is used: nozzle-contact allowed in ACM, TCP at contact point, need for compliance / force; record as decision |
| L-002 | Medium | L1 | DES-001 Rev 0.4 §1.3 cites WBS Rev 0.1, DES-003 Rev 0.2, REQ-002 "planned", TST-002 Rev 0.1 (current: 0.3 / 0.4 / Rev 0.1 exists / 0.2); §1.4 says D01–D19 of FR Rev 0.2, §5.1 says D01–D20 of Rev 0.3 | DES-001 Rev 0.4 | Update references in DES-001 Rev 0.5 |
| L-003 | Medium | L3 | DES-003 revision history Rev 0.3 says "units = m"; ADR-14 (same date) says mm for all project data | DES-003 Rev 0.4 "Revision History"; DES-001 ADR-14 | Correct the note in DES-003 Rev 0.5 |
| L-004 | Medium | L4 / L10 | pythonocc-core 7.9 (ADR-15 with Python 3.12; TST-002 E1; WBS 3.2) vs 8.0.1 / Python 3.10 env `awm_occ` (workspace `CLAUDE.md`, already marked [TBC]) | DES-001 ADR-15; TST-002; WBS; `CLAUDE.md` | Decide one version; update ADR-15 |
| L-005 | Low | L4 | Launch `sim` = URSim in DES-001 §4.2 (`awm_bringup`); `sim` = mock hardware (Sim-1) and URSim separate in `CLAUDE.md` | DES-001 §4.2; `CLAUDE.md` | Align names (e.g. `sim_mock`, `sim_ursim`) |
| L-006 | Medium | L1 / L2 | Claude Code prompts v1 / v2 point to DES-001 Rev 0.2 and DES-003 Rev 0.1, WPS "MIG + laser", Qt GUI out of scope | `Claude outputs\APLSMN601_ClaudeCode_Prompt_*.md` | Mark superseded by `CLAUDE.md` 2026-10-25 or issue prompt v3 |
| L-007 | Medium | L4 | Wrist-3 limit: TST-002 C3 "±180° of start" vs README open points "wrist-3 ±270°" | TST-002 §5; README (working) | Choose one value per purpose (spike criterion vs soft limit) and record in parameters register |
| L-008 | Low | L4 | Wobble range: slides "up to 5 mm"; DES-001 OI-18 "0–15 mm (distributor) vs max 5 mm (team note)" | LASER WELDING slide 2, 4; DES-001 OI-18 | Confirm in IPG manual; close OI-18 |
| L-009 | Medium | L8 | Question List now has Apillis comments for Q1–Q4 (UR5e + Jazzy; Phase 1 simulation; path from CAD STEP / STL; set-up and calibration at initialisation + sensor to detect the part); README says "no answers yet". Q6 says "run simulation with **152056** parts" — no STEP or drawing 152056 in the project (golden part in TST-002 is 564082). Q3 mentions STL while ADR-02 is STEP B-Rep (STL = Phase 2 fallback A16) | Question List (Google Sheet); README; DES-001 ADR-02 | Confirm part number 152056 vs 564082; update README / Q&A status |
| L-010 | Medium | L2 | Still MIG-first: Charter Rev 0.2, Kickoff Rev 0.1, schedule xlsx (MIG in rows 3.5, 5.3, 6.3, 7.2, 7.3; automatic seam detection still in row 8.3), REQ-002 Rev 0.1 (MIG components and operations). No CR file in `2…\06. CHANGE REQUESTS\` (README only) | PM-001, PM-002, PM-004 xlsx, REQ-002; PM-004 v2 §5 | Charter CR (owner Truong, due 2026-10-26 per PM-004 v2); REQ-002 Rev 0.2 laser-focused |
| L-011 | Low | L1 | Schedule "Docs Tracking" sheet lists PM-003 Rev 0.1 and REQ-002 / DES-001 as "Planned" | PM_Schedule.xlsx | Update with the schedule change |
| L-012 | Medium | L5 | DES-003 sheet "A0 Overview Phase 1" uses A1–A4 for a 4-block team pipeline (input "File STL"; pose X, Y, Z, Q1, Q2, Q3 + cf1–cf3) — clashes with IDEF0 A1–A9, and Q1–Q3 is ambiguous (rotation vector or 3 of 4 quaternion terms) | DES-003 Rev 0.4 | Rename blocks (e.g. P1–P4), state pose convention (UR rotation vector Rx, Ry, Rz?) and units |
| L-013 | Low | L8 | WBS 4.6 dictionary says criteria C1–C7; TST-002 Rev 0.2 moved C7 to the hardware stage and added C8 | WBS Rev 0.3; TST-002 Rev 0.2 | Update in WBS Rev 0.4 |
| L-014 | Low | L10 | Master `README.md` (last check 2026-10-12) is behind the working README (2026-10-22) | `{MASTER}\README.md`; project README | Sync after review |
| L-015 | Low | L10 | TST-002 records folder `4…\4. SOFTWARE\5. Spike\` does not exist yet; collision material sits in `4. Collision\`. P2 demo v1 / v2 exist in both `Claude outputs\` and `4. Collision\` | Folder listing 2026-10-27 | Decide one location for spike records |
| L-016 | Low | L7 | "AI Solution" collision concept (Collision Solution slide 6) has no doc number and no link to DES-001 / ADR-12 | Collision Solution slides | Backlog page + research question |
| L-017 | Info | L8 | TST-002 timebox 23–29 Oct; results due in Rev 0.3 on 2026-10-29; Sim-0 milestone 2026-10-30 | TST-002 §4; PM-004 v2 | Track as tasks; ingest results when available |

---

## Appendix B — Questions for Truong on this schema

Answered 2026-10-27:

1. **WIKI_ROOT location:** inside the ROS 2 workspace repo at `{WS}/docs/LLM_Wiki/` (Rev 0.2; repo used only by Truong), pushed with the code + weekly zip to Drive. Open: Git remote (must be private).
2. **"WSL synchronization":** document / data sync across the five locations of Section 8.5.
3. **Ingest mode:** one source at a time with Gate 1.

Still open:

4. **Wiki language:** English pages + Vietnamese chat summaries (default), or bilingual page summaries?
5. **Project docs:** copy `PROJECT.md` + `overview.md` into the claude.ai Project "Automatic Weldments" after approval?
6. **Cowork access:** a WSL folder is not reachable from Cowork unless it is connected (`\\wsl$\…`); otherwise Cowork works from the weekly zip / Drive copy. Acceptable?
