"""Turn the reviewed Rev 0.5 drafts into the Rev 0.5 release (decisions of Truong Hoang, 2026-10-29).

Usage (openpyxl from the conda env awm_occ):
    $AWM_OCC_PYTHON docs/generated/drafts/finalize_rev05.py <drafts_dir> <out_dir>
"""
import sys
from pathlib import Path

import openpyxl

DRAFTS, OUT = Path(sys.argv[1]), Path(sys.argv[2])
OWNER = "Software agent (Claude Code); direction + simulation tests: Truong Hoang"

# ---- DES-001 -------------------------------------------------------------------------------------
s = (DRAFTS / "APLSMN601-DES-001_Software_Architecture_Rev0.5_DRAFT.md").read_text(encoding="utf-8")


def rep(old, new):
    global s
    if s.count(old) != 1:
        sys.exit(f"not found once: {old[:80]!r}")
    s = s.replace(old, new)


rep("| Rev | 0.5 (draft) |", "| Rev | 0.5 |")
rep("| Status | Draft for review — prepared by the software agent from the decisions of 2026-10-27 (change proposal "
    "CP-9…CP-23) and the document review of 2026-10-29; not approved |",
    "| Status | Accepted by Truong Hoang on 2026-10-29 for team use (Engineering Manager approval pending). Prepared by "
    "the software agent from the decisions of 2026-10-27 (CP-9…CP-23) and the document review of 2026-10-29 |")
rep("| 0.5 (draft) | 2026-10-29 |", "| 0.5 | 2026-10-29 |")
rep("OI-13 and OI-20 closed, OI-22…OI-28 added |",
    "OI-13 and OI-20 closed, OI-22…OI-28 added; function owners and J2 strategy decided 2026-10-29 (OI-27, OI-28) |")
rep("| OI-27 | Which holders rotate with the part; positioner angle limits; J2 robot-only or Mode B | A49, D14 | "
    "Mechanical + software |",
    "| OI-27 | Which holders rotate with the part; positioner angle limits. J2: robot-only first, Mode B only if "
    "robot-only coverage < 100 % (decided 2026-10-29) | A49, D14 | Mechanical + software |")
rep("| OI-28 | Function owners in DES-003 are TBD (proposal in the review of 2026-10-29) | DES-003 | Engineering "
    "manager |",
    "| OI-28 | ~~Function owners~~ — closed 2026-10-29: all functions owned by the software agent (Claude Code); "
    "direction and simulation tests by Truong Hoang; execution on hardware (calibration, dry run, trial weld, "
    "inspection) assigned before the hardware stage | — | — |")
(OUT / "APLSMN601-DES-001_Software_Architecture_Rev0.5.md").write_text(s, encoding="utf-8")

# ---- DES-003 -------------------------------------------------------------------------------------
wb = openpyxl.load_workbook(DRAFTS / "APLSMN601-DES-003_Function_Register_Rev0.5_DRAFT.xlsx")
for ws in wb.worksheets:
    for row in ws.iter_rows(max_row=3):
        for c in row:
            if isinstance(c.value, str) and "Rev 0.5 DRAFT" in c.value:
                c.value = c.value.replace("Rev 0.5 DRAFT", "Rev 0.5")
fr = wb["Function Register"]
n = 0
for r in range(6, fr.max_row + 1):
    if fr.cell(r, 1).value:
        hw = fr.cell(r, 14).value == "Not in sim"
        fr.cell(r, 10).value = OWNER + ("; execution on hardware: TBD" if hw else "")
        n += 1
rh = wb["Revision History"]
assert rh["A9"].value == "0.5 DRAFT"
rh["A9"] = "0.5"
rh["D9"] = "Truong Hoang (prepared by the software agent)"
rh["C9"] = rh["C9"].value + (" Owners (2026-10-29): all functions by the software agent; direction and simulation "
                             "tests by Truong Hoang.")
last = max(r for r in range(20, rh.max_row + 1) if rh.cell(r, 1).value)
for i, (t, d, ref) in enumerate([
        ("Owners", "All functions owned by the software agent (Claude Code); Truong Hoang: direction and simulation "
                   "tests. Hardware execution assigned later.", "OI-28"),
        ("Seam J2", "Golden-part joint J2: robot-only first; positioner Mode B only if robot-only < 100 %.", "A49, OI-27"),
], start=1):
    r = last + i
    for c, v in enumerate([str(int(rh.cell(last, 1).value) + i), t, d, ref], start=1):
        rh.cell(r, c).value = v
        rh.cell(r, c)._style = rh.cell(last, c)._style
wb.calculation.fullCalcOnLoad = True
wb.save(OUT / "APLSMN601-DES-003_Function_Register_Rev0.5.xlsx")
print("owners set on", n, "functions; written to", OUT)
