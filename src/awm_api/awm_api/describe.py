"""``python3 -m awm_api.describe describe --frames <frames.yaml> --head-dir <dir>`` — print the cell URDF.

Runs in its own process (started by :mod:`awm_api.robot_model`) so the API process never imports xacro or the
planner. The URDF is the one the planner plans with: ``awm_path.description.load_planning_description`` with head
model ``hcad`` (one source, same xacro arguments and the same consistency checks against ``frames.yaml`` and the
positioner limits). Exit 1 with one ``error: …`` line on stderr when the description is refused.
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='awm_api.describe', description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('describe', help='render the cell URDF (head model hcad)')
    p.add_argument('--frames', required=True, help='cell/model/frames.yaml')
    p.add_argument('--head-dir', required=True, help='H-CAD head folder with head.yaml')
    args = ap.parse_args(list(argv) if argv is not None else None)
    try:
        from awm_path.description import load_planning_description
        desc = load_planning_description(args.frames, 'hcad', args.head_dir)
    except Exception as exc:  # noqa: BLE001 - every refusal is reported as one line
        sys.stderr.write(f'error: robot description refused: {exc}\n')
        return 1
    sys.stdout.write(desc.urdf + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
