"""``ros2 run awm_seam joint_seam`` / ``python3 -m awm_seam.joint_cli``: joint seams (D04 v2) and seam_axis.json.

Commands (lengths mm, angles deg; logs on stderr, one-line JSON summary on stdout, exit 1 on any error)::

    joint_seam build --job <job_dir> --picks picks.json [--pitch-mm 1.0] [--no-axis]
                     [--tol-r-spread-mm 0.1] [--tol-axis-angle-deg 0.5] [--occ-python-cmd CMD]
        picks.json = {"seams": [{"seam_id": "S01", "edges": ["E46"], "joint_type": "butt", "sequence": 1,
                                 "reversed": false, "start_offset_mm": 0, "start_point_mm": null, "arcs": 1,
                                 "arc_overlap_mm": 2.0, "positioner_mode": "sync" | "C" | "B",
                                 "positioner_angles_deg": null, "leg_mm": null, "note": ""}]}
        Writes the full seam list seams.json (replaces it), then seam_axis.json (needs placement.json and
        <repo>/cell/model/frames.yaml).
    joint_seam axis --job <job_dir> [tolerances]     rebuild seam_axis.json (after a placement / frames change)
    joint_seam show --job <job_dir>                  summary of seam_axis.json; refuses stale files

The OCC interpreter comes from ``--occ-python-cmd`` or the ``AWM_OCC_PYTHON`` environment variable.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional

from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.occ_bridge import GeometryCliError
from awm_core.repository import JobPaths
from awm_seam import joint_seams, seam_axis
from awm_seam.geometry_client import occ_sampler

LOG = logging.getLogger('awm_seam')


def summary(doc: Dict) -> Dict:
    """Per seam: points, spacing, r / axial / phi ranges, coaxial flag (for logs, awm_api and the report)."""
    out = {}
    for s in doc['seams']:
        phis = [p['phi_deg'] for p in s['points']]
        out[s['seam_id']] = {
            'joint_edge_id': s['joint_edge_id'], 'points': s['summary']['points'], 'spacing_mm': s['spacing_mm'],
            'length_mm': s['length_mm'], 'r_min_mm': s['summary']['r_min_mm'], 'r_max_mm': s['summary']['r_max_mm'],
            'axial_min_mm': s['summary']['axial_min_mm'], 'axial_max_mm': s['summary']['axial_max_mm'],
            'phi_start_deg': s['summary']['phi_start_deg'], 'phi_end_deg': s['summary']['phi_end_deg'],
            'phi_min_deg': min(phis), 'phi_max_deg': max(phis), 'coaxial': s['coaxial']['is_coaxial'],
            'axis_angle_deg': s['coaxial']['axis_angle_deg'], 'positioner_mode': s['positioner_mode'],
            'arcs': len(s['arcs'])}
    return out


def _axis(job: JobPaths, args) -> Dict:
    doc = seam_axis.build_seam_axis(job, args.tol_r_spread_mm, args.tol_axis_angle_deg)
    for sid, s in summary(doc).items():
        LOG.info('%s %s: %d points, r %.4f..%.4f mm, axial %.3f..%.3f mm, phi %.2f -> %.2f deg, coaxial %s',
                 sid, s['joint_edge_id'], s['points'], s['r_min_mm'], s['r_max_mm'], s['axial_min_mm'],
                 s['axial_max_mm'], s['phi_start_deg'], s['phi_end_deg'], s['coaxial'])
        if s['positioner_mode'] == 'C' and not s['coaxial']:
            LOG.warning('%s: positioner_mode C but the seam is not coaxial with the positioner axis; the planner '
                        'will reject it', sid)
    return doc


def _tolerances(p: argparse.ArgumentParser) -> None:
    p.add_argument('--tol-r-spread-mm', type=float, default=seam_axis.TOL_R_SPREAD_MM,
                   help='coaxial test: max radius spread (Assumption default)')
    p.add_argument('--tol-axis-angle-deg', type=float, default=seam_axis.TOL_AXIS_ANGLE_DEG,
                   help='coaxial test: max tangent angle to the plane normal to the axis (Assumption default)')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='joint_seam', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build', help='seams.json (+ seam_axis.json) from picked edge ids')
    b.add_argument('--job', required=True)
    b.add_argument('--picks', required=True, help='pick request JSON (see module help)')
    b.add_argument('--pitch-mm', type=float, default=joint_seams.MAX_PITCH_MM,
                   help=f'sampling pitch along the seam, <= {joint_seams.MAX_PITCH_MM} mm')
    b.add_argument('--no-axis', action='store_true', help='do not write seam_axis.json')
    b.add_argument('--occ-python-cmd', default=None, help='OCC interpreter (default: env AWM_OCC_PYTHON)')
    _tolerances(b)
    a = sub.add_parser('axis', help='rebuild seam_axis.json from seams.json, placement.json, frames.yaml')
    a.add_argument('--job', required=True)
    _tolerances(a)
    s = sub.add_parser('show', help='summary of seam_axis.json (refuses stale files)')
    s.add_argument('--job', required=True)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, stream=sys.stderr, format='[awm_seam] %(levelname)s: %(message)s')
    args = build_parser().parse_args(argv)
    job = JobPaths(args.job)
    try:
        if args.cmd == 'build':
            data = json.loads(Path(args.picks).read_text(encoding='utf-8'))
            requests = [joint_seams.JointSeamRequest.from_dict(r) for r in data['seams']]
            doc = joint_seams.build_seams(job, requests, args.pitch_mm, occ_sampler(args.occ_python_cmd))
            LOG.info('seams.json: %d seam(s) %s', len(doc['seams']), [s['seam_id'] for s in doc['seams']])
            result = {'seams_json': str(job.seams_json), 'seams': {}}
            if not args.no_axis:
                result['seam_axis_json'] = str(seam_axis.seam_axis_path(job))
                result['seams'] = summary(_axis(job, args))
        elif args.cmd == 'axis':
            result = {'seam_axis_json': str(seam_axis.seam_axis_path(job)), 'seams': summary(_axis(job, args))}
        else:
            result = {'seam_axis_json': str(seam_axis.seam_axis_path(job)),
                      'seams': summary(seam_axis.read_seam_axis(job))}
    except (joint_seams.SeamDefinitionError, seam_axis.SeamAxisError, SchemaValidationError, StaleInputError,
            GeometryCliError, FileNotFoundError, KeyError, ValueError) as exc:
        LOG.error('%s: %s', type(exc).__name__, exc)
        return 1
    sys.stdout.write(json.dumps(result) + '\n')
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
