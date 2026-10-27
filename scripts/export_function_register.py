#!/usr/bin/env python3
"""Export every sheet of the DES-003 Function Register (.xlsx) to Markdown tables.

Usage (openpyxl is installed in the conda env ``awm_occ``):
    $AWM_OCC_PYTHON scripts/export_function_register.py docs/reference/<register>.xlsx docs/generated

Writes ``function_register_<sheet>.md`` per sheet (values only, empty rows / columns dropped) and
removes older ``function_register_*.md`` files whose sheet no longer exists.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import openpyxl


def slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def cell_text(value: object) -> str:
    if value is None:
        return ""
    text = str(value).replace("|", "\\|").replace("\r", "")
    return " / ".join(part.strip() for part in text.split("\n") if part.strip())


def export(xlsx: Path, out_dir: Path) -> list[Path]:
    book = openpyxl.load_workbook(xlsx, data_only=True)
    rel_src = xlsx.as_posix()
    written: list[Path] = []
    for sheet in book.worksheets:
        rows = [[cell_text(v) for v in row] for row in sheet.iter_rows(values_only=True)]
        rows = [r for r in rows if any(r)]
        if not rows:
            continue
        used = [i for i in range(max(len(r) for r in rows)) if any(i < len(r) and r[i] for r in rows)]
        table = [[r[i] if i < len(r) else "" for i in used] for r in rows]
        header = "| " + " | ".join(f"c{n + 1}" for n in range(len(used))) + " |"
        lines = [
            f'# Function Register — sheet "{sheet.title}"',
            "",
            f"Exported from `{rel_src}` (values only). Do not edit; re-export with "
            "`scripts/export_function_register.py`.",
            "",
            header,
            "|" + "---|" * len(used),
        ]
        lines += ["| " + " | ".join(r) + " |" for r in table]
        path = out_dir / f"function_register_{slug(sheet.title)}.md"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        written.append(path)
    for old in out_dir.glob("function_register_*.md"):
        if old not in written:
            old.unlink()
    return written


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    for p in export(Path(sys.argv[1]), Path(sys.argv[2])):
        print(p)
