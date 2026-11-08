"""Fake engineering CLIs for the awm_api tests (no OCC, no ROS).

Stands in for three commands, chosen by the first non-``-m`` arguments:

* ``<occ interpreter> -m awm_geometry.cli import|export-view ...`` (set as the OCC interpreter command),
* ``joint_seam build --job J --picks P [--no-axis]`` (G1.2 seam CLI, not merged yet),
* ``plan --job J --collision off --out O [--seams S01 ...] [--head-dir D] [--travel-speed-mm-s V]`` (G1.3 planner),
* ``generate --job J`` (G1.5 program CLI: writes ``programs/P###/program.json``, prints ``result ok|refused``),
* ``describe --frames F [--head-dir D]`` (robot description render, prints a small URDF),
* ``replay --program F`` (G1.7 ``replay_program``): scenario from ``FAKE_REPLAY`` = ok (default) | nosim | noctl |
  nomock | start | partial | error; prints log lines and the result line ``replay {json}``, exit 0 / 2 / 1 like the
  real CLI.

Environment knobs: ``FAKE_SLEEP_S`` (sleep before finishing), ``FAKE_FAIL`` (exit 1 with a message),
``FAKE_ARGV_LOG`` (append the argv as one JSON line).
"""

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def progress(frac: float, text: str) -> None:
    sys.stderr.write(f'[fake] progress {frac:.2f} {text}\n')
    sys.stderr.flush()


def pause() -> None:
    total = float(os.environ.get('FAKE_SLEEP_S', '0'))
    steps = max(int(total / 0.05), 1) if total > 0 else 0
    for i in range(steps):
        time.sleep(0.05)
        if i % 10 == 0:
            progress(i / steps, f'working {i}/{steps}')


def fail_if_asked() -> None:
    msg = os.environ.get('FAKE_FAIL')
    if msg:
        sys.stderr.write(f'[fake] ERROR: {msg}\n')
        sys.exit(1)


def cmd_import(a) -> None:
    job = Path(a.job)
    (job / 'cache').mkdir(parents=True, exist_ok=True)
    shutil.copyfile(a.step, job / 'model.step')
    (job / 'part.json').write_text(json.dumps({
        'schema_version': '1.0', 'created_by': 'fake', 'created_at': '2026-11-06T00:00:00Z',
        'inputs': [{'name': 'model.step', 'sha256': sha(job / 'model.step')}],
        'part_no': a.part_no, 'part_rev': a.part_rev}))


FAKE_INFO = {'occ_version': '0.0-fake', 'awm_geometry': '0.0-fake'}
FAKE_CREATED_BY = 'awm_geometry 0.0-fake (pythonocc 0.0-fake)'


def cmd_export_view(a) -> None:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'view.glb').write_bytes(b'glTF-fake')
    (out / 'topology.json').write_text(json.dumps({
        'schema_version': '2.0', 'unit': 'mm', 'up_axis_arg': a.up_axis, 'created_by': FAKE_CREATED_BY,
        'inputs': [{'name': Path(a.step).name, 'sha256': sha(Path(a.step))}],
        'source': {'file': Path(a.step).name, 'component': a.component, 'up_axis': a.up_axis}}))


def _write(path, doc) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(doc if isinstance(doc, str) else json.dumps(doc))


def cmd_inspect_assembly(a) -> None:
    _write(a.out, {'schema_version': '2.0', 'source': {'file': Path(a.step).name, 'sha256': sha(Path(a.step))},
                   'components': [{'index': 0, 'parent': None, 'depth': 0, 'instance_name': '',
                                   'product_name': 'root', 'is_assembly': True, 'bbox_mm': None}]})


def cmd_export_cell_view(a) -> None:
    _write(Path(a.out) / 'cell_view.glb', 'glTF-fake-cell')


def cmd_pick_circle(a) -> None:
    _write(a.out, {'schema_version': '2.0', 'source': {'file': Path(a.step).name, 'sha256': sha(Path(a.step))},
                   'component_path': a.component, 'edge_id': a.edge, 'center_file_mm': [0.0, 0.0, 0.0],
                   'axis_file': [-1.0 if a.reverse else 1.0, 0.0, 0.0], 'radius_mm': 60.0,
                   'reversed': bool(a.reverse), 'method': 'pick'})


def cmd_apply_roles(a) -> None:
    _write(a.out, 'schema: cad_map/0.1\n')


def cmd_export_scene(a) -> None:
    _write(Path(a.repo) / 'cell' / 'model' / 'frames.yaml', 'unit: "mm"\n')


def cmd_export_head(a) -> None:
    _write(Path(a.repo) / 'cell' / 'model' / 'head' / 'head.yaml', f'tcp_edge: {a.tcp_edge}\n')


def cmd_job_placement(a) -> None:
    """FAKE_CHECK_STATUS: green | yellow | red (check 1 red, no placement) | red4 (only check 4 red, CP-62) |
    red14 (checks 1 and 4 red). FAKE_CHECK_HITS: comma-separated static names for check 4."""
    job = Path(a.repo) / 'jobs' / a.job
    status = os.environ.get('FAKE_CHECK_STATUS', 'green')
    inputs = [{'name': n, 'sha256': sha(job / n)} for n in ('model.step', 'roles.json', 'axis_pick.json')
              if (job / n).is_file()]
    c1_red, c4_red = status in ('red', 'red14'), status in ('red4', 'red14')
    hits = [{'static': s, 'rotating': 'Part1:1', 'count': 3, 'depth_mm': 4.5, 'at_x_mm': 1.0}
            for s in os.environ.get('FAKE_CHECK_HITS', 'Jig:1,Timing Belts:1').split(',')] if c4_red else []
    overall = 'red' if (c1_red or c4_red) else status
    _write(job / 'cad_check.json', {'schema_version': '2.0', 'created_by': 'fake', 'created_at': '2026-11-06T00:00:00Z',
                                    'inputs': inputs, 'status': overall, 'phi_deg': 0.0,
                                    'checks': [{'id': 1, 'name': 'positioner match',
                                                'status': 'red' if c1_red else 'green'},
                                               {'id': 4, 'name': 'swept rotating set vs static',
                                                'status': 'red' if c4_red else 'green', 'values': {'hits': hits},
                                                'note': ('static object(s) inside the volume swept by the rotating '
                                                         'set: ' + ', '.join(sorted({h['static'] for h in hits})))
                                                if hits else 'full turn checked'}]})
    if not c1_red:
        _write(job / 'placement.json', {'schema_version': '2.0', 'source': 'job_cad', 'verified': False,
                                        'inputs': inputs})


def cmd_build(a) -> None:
    job = Path(a.job)
    picks = json.loads(Path(a.picks).read_text())['seams']
    for p in picks:
        for e in p['edges']:
            if e == 'E999':
                sys.stderr.write('[fake] ERROR: edge E999 is not in topology.json\n')
                sys.exit(1)
    seams = []
    for p in picks:
        s = dict(p)
        s.update({'joint_edge_id': 'JE1', 'closed': True, 'length_mm': 40.0,
                  'geometry': {'spacing_mm': 10.0, 'points': [
                      {'s_mm': 10.0 * i, 'p_mm': [10.0 * i, 0.0, 0.0], 't': [1.0, 0.0, 0.0],
                       'n1': [0.0, 0.0, 1.0], 'n2': [0.0, 0.0, 1.0]} for i in range(5)]}})
        seams.append(s)
    (job / 'seams.json').write_text(json.dumps({'schema_version': '2.1', 'seams': seams}))
    if not a.no_axis:
        (job / 'seam_axis.json').write_text(json.dumps({
            'schema_version': '2.0',
            'inputs': [{'name': 'seams.json', 'sha256': sha(job / 'seams.json')},
                       {'name': 'placement.json', 'sha256': sha(job / 'placement.json')}]}))


def cmd_plan(a) -> None:
    Path(a.out).write_text(json.dumps({'schema_version': '2.1', 'collision_checked': False,
                                       'seams_arg': a.seams, 'travel_speed_arg': a.travel_speed_mm_s}))


def cmd_generate(a) -> None:
    job = Path(a.job)
    refuse = os.environ.get('FAKE_REFUSE')
    if refuse:
        print(f'result refused {refuse}')
        sys.exit(1)
    progs = job / 'programs'
    n = len([p for p in progs.iterdir() if p.is_dir()]) + 1 if progs.is_dir() else 1
    rev = f'P{n:03d}'
    out = progs / rev / 'program.json'
    out.parent.mkdir(parents=True)
    out.write_text(json.dumps({'schema_version': '2.1', 'program_rev': rev, 'collision_checked': False,
                               'cycle_time_s': 12.5, 'segments': [], 'events': [],
                               'inputs': [{'name': 'path.json',
                                           'sha256': os.environ.get('FAKE_PROGRAM_PATH_SHA') or sha(job / 'path.json')}]}))
    print(f'result ok program_rev={rev} out={out} cycle_time_s=12.5 collision_checked=false')


REPLAY_SEG = {'segment_id': 'G001', 'kind': 'air', 'seam_id': None, 'duration_s': 2.0, 'elapsed_s': 2.05,
              'start_deviation_deg': 0.0, 'end_deviation_deg': 0.0, 'max_tracking_error_deg': 0.058,
              'feedback_samples': 100}
REPLAY_CASES = {
    'nosim': (2, 'refused', '/controller_manager/list_hardware_components not available: cannot prove mock hardware'),
    'noctl': (1, 'error', '/robot_positioner_trajectory_controller/follow_joint_trajectory not available (launch '
                          'sim_mock with trajectory_controllers:=robot_positioner)'),
    'nomock': (2, 'refused', 'replay runs on mock hardware only (ADR-13); not mock: ur (ur_robot_driver/URPositionHardwareInterface)'),
    'start': (2, 'refused', 'G001: current state is not the segment start (tolerance 0.05 deg): shoulder_pan_joint '
                            '10.0000 deg (segment start 0.0000, off 10.0000)'),
    'partial': (2, 'partial', 'G002: current state is not the segment start (tolerance 0.05 deg): positioner_joint '
                              '3.0000 deg (segment start 0.0000, off 3.0000)'),
    'error': (1, 'error', 'G001: controller error -4 \'path tolerance violated\''),
}


def cmd_replay(a) -> int:
    case = os.environ.get('FAKE_REPLAY', 'ok')
    if case == 'noresult':
        print('[INFO] [awm_program_replay]: something else', flush=True)
        return 0
    summary = {'program': a.program, 'status': 'ok', 'segments': [], 'program_rev': 'P001',
               'controller_action': '/robot_positioner_trajectory_controller/follow_joint_trajectory'}
    print('[INFO] [awm_program_replay]: replay P001: 2 segment(s); weld events are not executed (motion only)',
          flush=True)
    if case == 'ok':
        summary['segments'] = [REPLAY_SEG, dict(REPLAY_SEG, segment_id='G002', kind='weld', seam_id='S01')]
        summary.update(final_deviation_deg=0.0, max_tracking_error_deg=0.058, final_state_deg={'positioner_joint': 0.0})
        code = 0
    else:
        code, status, reason = REPLAY_CASES[case]
        if case == 'partial':
            summary['segments'] = [REPLAY_SEG]
        summary.update(status=status, reason=reason)
        print(f'[ERROR] [awm_program_replay]: refused: {reason}', flush=True)
    print('replay ' + json.dumps(summary, sort_keys=True), flush=True)
    return code


def cmd_describe(a) -> None:
    head = a.head_dir
    sys.stdout.write(
        '<?xml version="1.0"?><robot name="fake"><link name="world"/>'
        f'<link name="head_body"><visual><geometry><mesh filename="file://{head}/head_body.stl" '
        'scale="0.001 0.001 0.001"/></geometry></visual></link>'
        '<link name="base_link"><visual><geometry><mesh '
        'filename="package://ur_description/meshes/ur5e/visual/base.dae"/></geometry></visual></link>'
        '</robot>\n')


def main() -> int:
    argv = sys.argv[1:]
    if argv[:1] == ['-m']:
        argv = argv[2:]
    log = os.environ.get('FAKE_ARGV_LOG')
    if log:
        with open(log, 'a') as fh:
            fh.write(json.dumps(argv) + '\n')
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('import')
    p.add_argument('--job', required=True)
    p.add_argument('--step', required=True)
    p.add_argument('--part-no', required=True)
    p.add_argument('--part-rev', required=True)
    p.add_argument('--component')
    p.add_argument('--components', action='store_true')
    p = sub.add_parser('export-view')
    p.add_argument('--step', required=True)
    p.add_argument('--job')
    p.add_argument('--component')
    p.add_argument('--components', action='store_true')
    p.add_argument('--up-axis', default='+Z')
    p.add_argument('--out', required=True)
    p = sub.add_parser('inspect-assembly')
    p.add_argument('--step', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--no-cylinders', action='store_true')
    p = sub.add_parser('export-cell-view')
    p.add_argument('--step', required=True)
    p.add_argument('--components', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--up-axis', required=True)
    p = sub.add_parser('pick-circle')
    p.add_argument('--step', required=True)
    p.add_argument('--component')
    p.add_argument('--components', action='store_true')
    p.add_argument('--edge', required=True)
    p.add_argument('--reverse', action='store_true')
    p.add_argument('--up-axis', required=True)
    p.add_argument('--out', required=True)
    p = sub.add_parser('apply-roles')
    for k in ('--step', '--tree', '--roles', '--axis', '--out'):
        p.add_argument(k, required=True)
    p = sub.add_parser('export-scene')
    p.add_argument('--step', required=True)
    p.add_argument('--map', required=True)
    p.add_argument('--repo', required=True)
    p.add_argument('--job')
    p = sub.add_parser('export-head')
    p.add_argument('--step', required=True)
    p.add_argument('--map', required=True)
    p.add_argument('--repo', required=True)
    p.add_argument('--tcp-component')
    p.add_argument('--tcp-edge')
    p.add_argument('--tcp-reverse', action='store_true')
    p.add_argument('--stickout-mm', type=float)
    p.add_argument('--taught-by')
    p = sub.add_parser('job-placement')
    p.add_argument('--repo', required=True)
    p.add_argument('--job', required=True)
    p = sub.add_parser('build')
    p.add_argument('--job', required=True)
    p.add_argument('--picks', required=True)
    p.add_argument('--no-axis', action='store_true')
    p = sub.add_parser('plan')
    p.add_argument('--job', required=True)
    p.add_argument('--collision', required=True, choices=['off'])
    p.add_argument('--out', required=True)
    p.add_argument('--seams', nargs='*')
    p.add_argument('--head-dir')
    p.add_argument('--travel-speed-mm-s', type=float)
    p = sub.add_parser('generate')
    p.add_argument('--job', required=True)
    p = sub.add_parser('describe')
    p.add_argument('--frames', required=True)
    p.add_argument('--head-dir')
    sub.add_parser('info')
    p = sub.add_parser('replay')
    p.add_argument('--program', required=True)
    p = sub.add_parser('sleep')
    a = ap.parse_args(argv)
    if a.cmd == 'info':
        print(json.dumps(FAKE_INFO))
        return 0
    if a.cmd == 'replay':
        pause()
        return cmd_replay(a)
    if a.cmd == 'describe':
        fail_if_asked()
        cmd_describe(a)
        return 0
    progress(0.0, f'{a.cmd} start')
    pause()
    fail_if_asked()
    {'import': cmd_import, 'export-view': cmd_export_view, 'build': cmd_build, 'plan': cmd_plan,
     'generate': cmd_generate, 'sleep': lambda _a: None, 'inspect-assembly': cmd_inspect_assembly,
     'export-cell-view': cmd_export_cell_view, 'pick-circle': cmd_pick_circle, 'apply-roles': cmd_apply_roles,
     'export-scene': cmd_export_scene, 'export-head': cmd_export_head,
     'job-placement': cmd_job_placement}[a.cmd](a)
    progress(1.0, f'{a.cmd} done')
    print(json.dumps({'ok': True}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
