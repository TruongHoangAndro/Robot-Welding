# Log

Append-only. Filter: `grep "^## \[" wiki/log.md | tail -10`. Types: ingest · query · lint · decision · code · sync · task · schema.

## [2026-10-27] schema | PROJECT.md Rev 0.1 approved for first use
- Decided by Truong: WIKI_ROOT = Git repo in WSL + weekly zip backup to Drive; "WSL synchronization" = document / data sync across 5 locations (PROJECT.md §8.5); ingest = one source at a time with Gate 1
- Open: wiki language (Appendix B Q4), copy to claude.ai Project (Q5), Cowork access to WSL (Q6), Git path and remote

## [2026-10-27] lint | First full read of the project (seed, no wiki pages yet)
- Read: master folder, Google Slides / Sheets / Docs, LLM Wiki idea file
- Findings: L-001…L-017 → [[lint/open-issues]] (1 High: L-001 nozzle contact vs TCP / ACM)
- Not read yet: code in `Claude outputs/` and Tac demo zips (batch 4)

## [2026-10-27] schema | PROJECT.md Rev 0.2 — wiki moved into {WS}/docs/LLM_Wiki
- Decided by Truong: one Git repo for code + wiki (repo used only by Truong)
- New rules (PROJECT.md §4.1): read-only for coding sessions, separate `wiki:` commits, COLCON_IGNORE, private remote only
- Pending: patch of the workspace CLAUDE.md (T-011)

## [2026-10-27] sync | Skeleton created (APLSMN601_WikiSkeleton_v0.2.zip)
- Created: PROJECT.md, CLAUDE.md, COLCON_IGNORE, .gitignore, raw/_manifest.md (30 sources), wiki/overview, index, log, glossary, registers (8, empty), _templates (5), todo-tasks/open, lint/open-issues
- Next: T-001 extract into {WS}/docs/ + first `wiki:` commit (Truong) → T-011 → T-002 ingest batch 1, starting with DES-001 Rev 0.4
