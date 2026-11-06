"""Fake engineering CLIs for the awm_api tests (no OCC, no ROS).

Stands in for three commands, chosen by the first non-``-m`` arguments:

* ``<occ interpreter> -m awm_geometry.cli import|export-view ...`` (set as the OCC interpreter command),
* ``joint_seam build --job J --picks P [--no-axis]`` (G1.2 seam CLI, not merged yet),
* ``plan --job J --collision off --out O [--seams S01 ...] [--head-dir D] [--travel-speed-mm-s V]`` (G1.3 planner),
* ``generate --job J`` (G1.5 program CLI: writes ``programs/P###/program.json``, prints ``result ok|refused``),
* ``describe --frames F [--head-dir D]`` (robot description render, prints a small URDF).

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


def cmd_export_view(a) -> None:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'view.glb').write_bytes(b'glTF-fake')
    (out / 'topology.json').write_text(json.dumps({'schema_version': '2.0', 'unit': 'mm',
                                                   'up_axis_arg': a.up_axis}))


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
                               'inputs': [{'name': 'path.json', 'sha256': sha(job / 'path.json')}]}))
    print(f'result ok program_rev={rev} out={out} cycle_time_s=12.5 collision_checked=false')


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
    p = sub.add_parser('export-view')
    p.add_argument('--step', required=True)
    p.add_argument('--job')
    p.add_argument('--component')
    p.add_argument('--up-axis', default='+Z')
    p.add_argument('--out', required=True)
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
    p = sub.add_parser('sleep')
    a = ap.parse_args(argv)
    if a.cmd == 'describe':
        fail_if_asked()
        cmd_describe(a)
        return 0
    progress(0.0, f'{a.cmd} start')
    pause()
    fail_if_asked()
    {'import': cmd_import, 'export-view': cmd_export_view, 'build': cmd_build, 'plan': cmd_plan,
     'generate': cmd_generate, 'sleep': lambda _a: None}[a.cmd](a)
    progress(1.0, f'{a.cmd} done')
    print(json.dumps({'ok': True}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
