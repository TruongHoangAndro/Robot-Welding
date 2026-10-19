"""awm_geometry CLI (runs in the conda env ``awm_occ``).

Commands (all lengths in metres):
  info                                       OCC version as JSON
  import --job DIR --step FILE --part-no N --part-rev R [--linear-deflection-m D]
                                             A11-A14: STEP -> model.step copy, model.brep, mesh.stl,
                                             edges.json, part.json
  sample --job DIR --spacing-m S             A41 input: seams.json -> cache/seam_geometry.json
  make-test-part --kind K --out FILE.step [--picks FILE.json]

Logs go to stderr; a one-line JSON summary goes to stdout.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import shutil
import sys
from pathlib import Path
from typing import List, Optional

import awm_geometry
from awm_core import jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths

LOG = logging.getLogger('awm_geometry')


def _created_by() -> str:
    import OCC
    return f'awm_geometry {awm_geometry.__version__} (pythonocc {OCC.VERSION})'


def cmd_info(_args) -> dict:
    import OCC
    return {'occ_version': OCC.VERSION, 'awm_geometry': awm_geometry.__version__}


def cmd_import(args) -> dict:
    from awm_geometry import occ_io, topology

    job = JobPaths(args.job)
    step_src = Path(args.step)
    if not step_src.is_file():
        raise occ_io.GeometryError(f'STEP file not found: {step_src}')
    job.dir.mkdir(parents=True, exist_ok=True)
    job.cache_dir.mkdir(exist_ok=True)
    if step_src.resolve() != job.model_step.resolve():
        if job.model_step.exists():
            job.model_step.chmod(0o644)
        shutil.copyfile(step_src, job.model_step)
        job.model_step.chmod(0o444)
    LOG.info('A11 reading %s', job.model_step)
    shape, unit = occ_io.read_step(job.model_step)
    LOG.info('A12 check & heal')
    shape, healed, checks = topology.check_and_heal(shape, unit)
    failed = [c for c in checks if not c.ok and c.name in ('brep_valid', 'has_solid')]
    occ_io.write_brep(shape, job.brep_cache)
    LOG.info('A13 tessellate (linear deflection %.5f m)', args.linear_deflection_m)
    triangles = occ_io.tessellate_to_stl(shape, job.mesh_stl, args.linear_deflection_m,
                                         math.radians(args.angular_deflection_deg))
    LOG.info('A21 edge catalogue')
    catalogue = topology.edge_catalogue(shape)
    edges_doc = jsonio.make_header(_created_by(), [job.model_step], job.dir)
    edges_doc.update(catalogue)
    jsonio.write_json(job.edges_json, edges_doc, 'edges')
    lo, hi = topology.bbox(shape)
    part = jsonio.make_header(_created_by(), [job.model_step], job.dir)
    part.update({
        'part_no': args.part_no, 'part_rev': args.part_rev,
        'step_file': step_src.name, 'step_sha256': sha256_file(job.model_step),
        'unit_source': unit, 'bbox_m': {'min': lo, 'max': hi},
        'counts': topology.counts(shape),
        'checks': [c.as_dict() for c in checks], 'healed': healed,
        'mesh': {'file': 'cache/mesh.stl', 'sha256': sha256_file(job.mesh_stl),
                 'linear_deflection_m': args.linear_deflection_m,
                 'angular_deflection_rad': math.radians(args.angular_deflection_deg),
                 'triangles': triangles},
        'brep_cache': {'file': 'cache/model.brep', 'sha256': sha256_file(job.brep_cache)},
        'edges_file': {'file': 'cache/edges.json', 'sha256': sha256_file(job.edges_json)},
    })
    jsonio.write_json(job.part_json, part, 'part')
    LOG.info('A14 part record written: %s', job.part_json)
    if failed:
        raise occ_io.GeometryError('model check failed: ' + '; '.join(
            f'{c.name}: {c.detail}' for c in failed))
    return {'part_json': str(job.part_json), 'edges': len(catalogue['edges']),
            'faces': len(catalogue['faces']), 'triangles': triangles,
            'checks_failed': [c.name for c in checks if not c.ok]}


def cmd_sample(args) -> dict:
    from awm_geometry import occ_io, sampling

    job = JobPaths(args.job)
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    seams = jsonio.read_json(job.seams_json, 'seams', base_dir=job.dir)
    if sha256_file(job.brep_cache) != part['brep_cache']['sha256']:
        raise occ_io.GeometryError('cache/model.brep does not match part.json; re-run import')
    shape = occ_io.read_brep(job.brep_cache)
    out = []
    for seam in seams['seams']:
        LOG.info('A41 sampling %s (%d edge(s))', seam['seam_id'], len(seam['edges']))
        out.append(sampling.sample_seam(shape, seam, args.spacing_m))
    doc = jsonio.make_header(_created_by(), [job.part_json, job.seams_json], job.dir)
    doc.update({'spacing_m': args.spacing_m, 'seams': out})
    jsonio.write_json(job.seam_geometry_json, doc, 'seam_geometry')
    return {'seam_geometry_json': str(job.seam_geometry_json),
            'seams': {s['seam_id']: len(s['points']) for s in out}}


def cmd_make_test_part(args) -> dict:
    from awm_geometry import occ_io, testparts

    if args.kind not in testparts.PARTS:
        raise occ_io.GeometryError(f'unknown test part {args.kind!r}; '
                                   f'choose from {sorted(testparts.PARTS)}')
    shape, picks = testparts.PARTS[args.kind]()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    occ_io.write_step_mm(shape, out)
    result = {'step': str(out)}
    if args.picks:
        Path(args.picks).write_text(json.dumps(picks, indent=2) + '\n')
        result['picks'] = args.picks
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='awm_geometry', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-v', '--verbose', action='store_true')
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('info')
    p = sub.add_parser('import')
    p.add_argument('--job', required=True)
    p.add_argument('--step', required=True)
    p.add_argument('--part-no', required=True)
    p.add_argument('--part-rev', required=True)
    p.add_argument('--linear-deflection-m', type=float, default=1.0e-4)
    p.add_argument('--angular-deflection-deg', type=float, default=10.0)
    p = sub.add_parser('sample')
    p.add_argument('--job', required=True)
    p.add_argument('--spacing-m', type=float, required=True)
    p = sub.add_parser('make-test-part')
    p.add_argument('--kind', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--picks')
    return parser


COMMANDS = {'info': cmd_info, 'import': cmd_import, 'sample': cmd_sample,
            'make-test-part': cmd_make_test_part}


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, stream=sys.stderr,
                        format='[awm_geometry] %(levelname)s: %(message)s')
    # OCCT prints transfer statistics on fd 1; keep stdout for the JSON summary only.
    sys.stdout.flush()
    real_stdout = os.dup(1)
    os.dup2(2, 1)
    try:
        result = COMMANDS[args.cmd](args)
    except Exception as exc:  # noqa: BLE001 - CLI boundary: report and exit non-zero
        LOG.error('%s: %s', type(exc).__name__, exc)
        if args.verbose:
            LOG.exception('traceback')
        return 1
    finally:
        sys.stdout.flush()
        os.dup2(real_stdout, 1)
        os.close(real_stdout)
    sys.stdout.write(json.dumps(result) + '\n')
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
