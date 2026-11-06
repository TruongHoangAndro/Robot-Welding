"""awm_geometry CLI (runs in the conda env ``awm_occ``).

Commands (lengths in metres, except export-view: mm per the D20 contract):
  info                                       OCC version as JSON
  import --job DIR --step FILE --part-no N --part-rev R [--component NAME] [--linear-deflection-m D]
                                             A11-A14: STEP -> model.step copy, model.brep, mesh.stl,
                                             edges.json, part.json
  sample --job DIR --spacing-m S             A41 input: seams.json -> cache/seam_geometry.json
  make-test-part --kind K --out FILE.step [--picks FILE.json]
  sample-joints --job DIR --request FILE.json --out FILE.json
                                             M2 (G1.2): joint seam samples in mm (points, tangent, joint-aware
                                             n1 / n2) for awm_seam; request / output format in joint_seam.py
  export-view --step FILE [--job DIR] [--component NAME] [--up-axis +Y] --out DIR
                                             A13 D20 view model in mm: DIR/view.glb + DIR/topology.json
                                             (contract docs/generated/contracts/D20_view_model.md)
  export-scene --step FILE --map MAP.yaml --repo DIR --job JOB [--chord-near-mm 0.1]
                                             A15 / A18 spike scene (mm): cell/model hulls + frames.yaml +
                                             scene_manifest.json; job import, D20, collision mesh, placement
  export-cell-view --step FILE --components FILE.json --out DIR --up-axis AXIS
                                             S10-lite display model DIR/cell_view.glb (one node per leaf
                                             component, file frame, mm; contract S10_lite_cell_model.md)
  export-head --step FILE --map MAP.yaml --repo DIR
                                             A18 H-CAD head (mm, tool0 frame): cell/model/head/ meshes +
                                             hulls + head.yaml (tool0 / tcp_laser estimate, S.5c)
  pick-circle --step FILE [--component NAME | --components] --edge E<n> [--reverse] --up-axis A --out FILE.json
                                             CP-60 / CP-61: centre, axis, radius of a circular D20 edge (file mm)
  inspect-assembly --step FILE --out FILE.json [--no-cylinders]
                                             A18 input: component tree of a STEP assembly (paths,
                                             names, placements 4x4 and bboxes in mm, file frame)

Logs go to stderr; a one-line JSON summary goes to stdout.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

import awm_geometry
from awm_core import jsonio, units
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
    from awm_geometry import part as part_mod

    job = JobPaths(args.job)
    copy_step(Path(args.step), job)
    LOG.info('A11 reading %s%s', job.model_step,
             f' (component {args.component})' if args.component else '')
    LOG.info('A12 check & heal')
    loaded = part_mod.load_part(job.model_step, args.component, components=args.components)
    return write_part_record(job, Path(args.step).name, loaded, args.part_no, args.part_rev,
                             args.linear_deflection_m, args.angular_deflection_deg)


def copy_step(step_src: Path, job: JobPaths) -> None:
    """A11: copy the STEP file into the job as read-only ``model.step`` (D01)."""
    from awm_geometry import occ_io

    if not step_src.is_file():
        raise occ_io.GeometryError(f'STEP file not found: {step_src}')
    job.dir.mkdir(parents=True, exist_ok=True)
    job.cache_dir.mkdir(exist_ok=True)
    if step_src.resolve() != job.model_step.resolve():
        if job.model_step.exists():
            job.model_step.chmod(0o644)
        shutil.copyfile(step_src, job.model_step)
        job.model_step.chmod(0o444)


def write_part_record(job: JobPaths, step_name: str, loaded, part_no: str, part_rev: str,
                      linear_deflection_m: float, angular_deflection_deg: float) -> dict:
    """A12-A14 outputs of a loaded part: model.brep, mesh.stl, edges.json, part.json."""
    from awm_geometry import occ_io, topology

    shape, unit, healed, checks = loaded.shape, loaded.unit, loaded.healed, loaded.checks
    failed = [c for c in checks if not c.ok and c.name in ('brep_valid', 'has_solid')]
    occ_io.write_brep(shape, job.brep_cache)
    LOG.info('A13 tessellate (linear deflection %.5f m)', linear_deflection_m)
    triangles = occ_io.tessellate_to_stl(shape, job.mesh_stl, linear_deflection_m,
                                         units.deg_to_rad(angular_deflection_deg))
    LOG.info('A21 edge catalogue')
    catalogue = topology.edge_catalogue(shape)
    edges_doc = jsonio.make_header(_created_by(), [job.model_step], job.dir)
    edges_doc.update(catalogue)
    jsonio.write_json(job.edges_json, edges_doc, 'edges')
    lo, hi = topology.bbox(shape)
    part = jsonio.make_header(_created_by(), [job.model_step], job.dir)
    part.update({
        'part_no': part_no, 'part_rev': part_rev,
        'step_file': step_name, 'step_sha256': sha256_file(job.model_step),
        'unit_source': unit, 'bbox_m': {'min': lo, 'max': hi},
        'counts': topology.counts(shape),
        'checks': [c.as_dict() for c in checks], 'healed': healed,
        'mesh': {'file': 'cache/mesh.stl', 'sha256': sha256_file(job.mesh_stl),
                 'linear_deflection_m': linear_deflection_m,
                 'angular_deflection_rad': units.deg_to_rad(angular_deflection_deg),
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


def cmd_sample_joints(args) -> dict:
    from awm_geometry import joint_seam

    job = JobPaths(args.job)
    request = json.loads(Path(args.request).read_text(encoding='utf-8'))
    doc = joint_seam.sample_joints(job, request, _created_by())
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc) + '\n')
    return {'out': str(out), 'seams': {s['seam_id']: len(s['points']) for s in doc['seams']}}


def cmd_export_view(args) -> dict:
    from awm_geometry import part, view

    step = Path(args.step)
    job = Path(args.job) if args.job else None
    view.source_name(step, job)  # rule 7: fail early if the STEP is not inside the job
    LOG.info('A11 reading %s%s', step, f' (component {args.component})' if args.component else '')
    loaded = part.load_part(step, args.component, components=args.components)
    return view.export(loaded, step, args.component, args.up_axis, Path(args.out),
                       _created_by(), job)


def cmd_inspect_assembly(args) -> dict:
    from awm_geometry import assembly

    step = Path(args.step)
    LOG.info('A18 reading assembly %s', step)
    components, unit = assembly.component_tree(step, cylinders=not args.no_cylinders)
    scaled = [c['index'] for c in components
              if abs(abs(np.linalg.det(np.asarray(c['local_placement_mm'])[:3, :3])) - 1.0) > 1e-9]
    if scaled:
        LOG.warning('%d placement(s) are not rigid (scale != 1): %s', len(scaled), scaled[:10])
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = dict(jsonio.make_header(_created_by(), [step], out.parent), schema_version='2.0')
    doc.update({'source': {'file': step.name, 'sha256': sha256_file(step)},
                'unit_source': unit, 'units': {'length': 'mm'}, 'frame': 'file',
                'components': components})
    out.write_text(json.dumps(doc, indent=1) + '\n')
    roots = [c for c in components if c['parent'] is None]
    return {'out': str(out), 'components': len(components),
            'solids': sum(c['solids'] for c in roots), 'unit_source': unit}


def cmd_export_scene(args) -> dict:
    from awm_geometry import scene

    return scene.export_scene(Path(args.step), Path(args.map), Path(args.repo), args.job,
                              chord_near_mm=args.chord_near_mm, chord_far_mm=args.chord_far_mm,
                              padding_mm=args.padding_mm, created_by=_created_by())


def cmd_export_head(args) -> dict:
    from awm_geometry import head

    return head.export_head(Path(args.step), Path(args.map), Path(args.repo), _created_by())


def cmd_pick_circle(args) -> dict:
    from awm_geometry import circle_pick

    return circle_pick.pick(Path(args.step), args.edge, args.up_axis, Path(args.out), _created_by(),
                            component=args.component, components=args.components, reverse=args.reverse)


def cmd_export_cell_view(args) -> dict:
    from awm_geometry import cell_view

    return cell_view.export(Path(args.step), Path(args.components), Path(args.out), args.up_axis,
                            _created_by())


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
    p.add_argument('--component', default=None,
                   help='assembly component name (default: the whole file); same as export-view')
    p.add_argument('--components', action='store_true',
                   help='CP-60: whole file read per assembly component (same flag for export-view and pick-circle)')
    p.add_argument('--linear-deflection-m', type=float, default=1.0e-4)
    p.add_argument('--angular-deflection-deg', type=float, default=10.0)
    p = sub.add_parser('sample')
    p.add_argument('--job', required=True)
    p.add_argument('--spacing-m', type=float, required=True)
    p = sub.add_parser('make-test-part')
    p.add_argument('--kind', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--picks')
    p = sub.add_parser('sample-joints')
    p.add_argument('--job', required=True)
    p.add_argument('--request', required=True, help='sampler request JSON written by awm_seam')
    p.add_argument('--out', required=True, help='output JSON (mm, part frame)')
    p = sub.add_parser('export-view')
    p.add_argument('--step', required=True)
    p.add_argument('--job', default=None,
                   help='job folder: source.file / inputs relative to it (STEP must be inside)')
    p.add_argument('--component', default=None,
                   help='assembly component name (default: the whole file)')
    p.add_argument('--components', action='store_true',
                   help='CP-60: whole file per assembly component; solids carry component_index / component_path')
    p.add_argument('--up-axis', default='+Z', choices=['+X', '-X', '+Y', '-Y', '+Z', '-Z'],
                   help='up axis of the source file (recorded only; no rotation is applied)')
    p.add_argument('--out', required=True)
    p = sub.add_parser('inspect-assembly')
    p.add_argument('--step', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--no-cylinders', action='store_true',
                   help='skip the largest-cylinder axis of each leaf part')
    p = sub.add_parser('export-scene')
    p.add_argument('--step', required=True, help='cell CAD STEP assembly (D21)')
    p.add_argument('--map', required=True, help='role map cad_map.yaml (component -> role, frames)')
    p.add_argument('--repo', required=True, help='repository root (cell/model and jobs/<job> are written)')
    p.add_argument('--job', required=True, help='job folder name <PartNo>_<PartRev>')
    p.add_argument('--chord-near-mm', type=float, default=0.1,
                   help='part collision mesh chord on the faces at joint edges (DES-001 5.4)')
    p.add_argument('--chord-far-mm', type=float, default=0.5,
                   help='part collision mesh chord elsewhere')
    p.add_argument('--padding-mm', type=float, default=3.0,
                   help='padding of the cell objects (D16 padding_mm)')
    p = sub.add_parser('export-head')
    p.add_argument('--step', required=True, help='head CAD STEP assembly (robot arm + mount + LightWELD head)')
    p.add_argument('--map', required=True, help='head map cad_map_head.yaml (components, links, frame rules)')
    p.add_argument('--repo', required=True, help='repository root (cell/model/head is written)')
    p = sub.add_parser('pick-circle')
    p.add_argument('--step', required=True)
    group = p.add_mutually_exclusive_group()
    group.add_argument('--component', default=None, help='same component as the export-view the edge id came from')
    group.add_argument('--components', action='store_true', help='same flag as the export-view the edge id came from')
    p.add_argument('--edge', required=True, help='edge id E<n> of that D20 view')
    p.add_argument('--reverse', action='store_true', help='flip the circle axis (UI Reverse button)')
    p.add_argument('--up-axis', required=True, choices=['+X', '-X', '+Y', '-Y', '+Z', '-Z'])
    p.add_argument('--out', required=True, help='axis_pick.json (v2/axis_pick)')
    p = sub.add_parser('export-cell-view')
    p.add_argument('--step', required=True, help='cell CAD STEP assembly')
    p.add_argument('--components', required=True, help='cell_components.json of inspect-assembly')
    p.add_argument('--out', required=True, help='folder for cell_view.glb')
    p.add_argument('--up-axis', required=True, choices=['+X', '-X', '+Y', '-Y', '+Z', '-Z'],
                   help='up axis of the file (recorded only; no rotation is applied); no default: it is cell data')
    return parser


COMMANDS = {'info': cmd_info, 'import': cmd_import, 'sample': cmd_sample,
            'make-test-part': cmd_make_test_part, 'sample-joints': cmd_sample_joints, 'export-view': cmd_export_view,
            'inspect-assembly': cmd_inspect_assembly, 'export-scene': cmd_export_scene,
            'export-cell-view': cmd_export_cell_view, 'pick-circle': cmd_pick_circle, 'export-head': cmd_export_head}


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
