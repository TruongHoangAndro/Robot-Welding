# WP-S S.4 Part C — open items carried into S.5 (planner, 2026-11-04)

Recorded from the review of `ws-s4c-checks`; no code change in S.4.

| # | Item | Status |
|---|---|---|
| m4 | F9 rule (a) accepts `tcp_laser` anywhere on n1 ∪ n2 within 0.15 mm, not only near the seam edge. S.5 builds the TCP from the seam frame (on the edge), so this does not change S.5 results; whether rule (a) should also bound the distance to the seam edge is open | Open (S.5) |
| m6 | F5 clearance links = head body, mount, conduit (TST-002 F5 / C2, DES-001 Rev 0.6 §5.4). The S.4 task plan C2 also names `nozzle_body` (which ends 5 mm from the contact point, so a 5 mm clearance on it would reject every weld pose) | Awaiting Truong |
| m9 | C5: about 17.6 s for one J2 half arc (37 points, Ψ-FREE, `move_group` backend, without DP); two arcs projected at about 35 s against the 30 s budget per seam. Projection only; S.5 measures C5 with the DP | Open (S.5) |
| — | F9 at J1: nipple thread flanks come within s_c of the Ø 20 mm nozzle disc at several J1 points (G2 pose Z5 uses one). Input to S.6 / C9 and to the nozzle shape (I2) | Recorded |
| — | F9 "within tolerance" = 0.15 mm, Assumption (TST-002 gives no value) | Assumption |

## Decision 2026-11-04 (Truong Hoang)

- **G2 run 4 accepted** as the TST-002 G2 entry gate result for S.5 (record `g2_log.json`, commit cfd1890,
  `dirty: false`, H-BOX head model). Runs 1–3 stay as history. Open: m6 clearance link set (TST-002 vs plan),
  contact-zone pose reading (Pismo plate, holder Part3, Z5 on the J1 thread) — not decided yet.
