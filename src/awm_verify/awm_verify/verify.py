"""A71: verify a program revision in simulation and write the D15 report.

Checks: inputs (hashes), continuity, joint position / velocity limits, events at rest points,
collision of every trajectory state (interpolated to <= 0.02 rad), replay on mock hardware.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from awm_core import jsonio
from awm_core.calibration import load_active
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths
from awm_path.moveit_client import MoveItClient
from awm_path.params import ChainParams
from awm_path.planner import load_part_into_scene
from awm_program.generator import VEL_TOL, velocity_limits
from awm_program.timing import max_velocity_ratio
from awm_verify import __version__
from awm_verify.replay import Replayer

CREATED_BY = f'awm_verify {__version__}'
LOG = logging.getLogger('awm_verify')
COLLISION_STEP_RAD = 0.02


def _degrees_loader():
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor('!degrees', lambda loader, node: np.radians(
        float(loader.construct_scalar(node))))
    return Loader


def joint_position_limits(joint_names: List[str], ur_type: str = 'ur5e') -> np.ndarray:
    """[6,2] position limits from ur_description (single source of the robot limits)."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('ur_description')) / 'config' / ur_type / \
        'joint_limits.yaml'
    data = yaml.load(path.read_text(), Loader=_degrees_loader())['joint_limits']
    return np.array([[data[j]['min_position'], data[j]['max_position']] for j in joint_names])


def check_continuity(doc: Dict) -> Tuple[bool, str]:
    segs = doc['segments']
    home = np.asarray(doc['home_joints'])
    problems = []
    if np.max(np.abs(np.asarray(segs[0]['points'][0]['q_rad']) - home)) > 1e-6:
        problems.append('program does not start at home')
    if np.max(np.abs(np.asarray(segs[-1]['points'][-1]['q_rad']) - home)) > 1e-6:
        problems.append('program does not end at home')
    for a, b in zip(segs, segs[1:]):
        gap = np.max(np.abs(np.asarray(a['points'][-1]['q_rad']) -
                            np.asarray(b['points'][0]['q_rad'])))
        if gap > 1e-6:
            problems.append(f'{a["segment_id"]} -> {b["segment_id"]} gap {gap:.2e} rad')
    for s in segs:
        t = [p['t_s'] for p in s['points']]
        if t[0] != 0.0 or any(b <= a for a, b in zip(t, t[1:])):
            problems.append(f'{s["segment_id"]} times not strictly increasing from 0')
    return not problems, '; '.join(problems) or f'{len(segs)} segments chained, home to home'


def check_limits(doc: Dict, params: ChainParams) -> Tuple[bool, str]:
    limits = joint_position_limits(doc['joint_names'])
    problems = []
    worst = 0.0
    for s in doc['segments']:
        q = np.array([p['q_rad'] for p in s['points']])
        t = np.array([p['t_s'] for p in s['points']])
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            problems.append(f'{s["segment_id"]} outside joint position limits')
        ratio, joint = max_velocity_ratio(q, t, velocity_limits(params, s['kind']))
        worst = max(worst, ratio)
        if ratio > VEL_TOL:
            problems.append(f'{s["segment_id"]} ({s["kind"]}) joint {doc["joint_names"][joint]} '
                            f'at {ratio:.2f} x its limit')
    return not problems, '; '.join(problems) or f'max velocity {worst:.0%} of the segment limit'


def check_events(doc: Dict) -> Tuple[bool, str]:
    by_id = {s['segment_id']: s for s in doc['segments']}
    problems = []
    seams = {s['seam_id'] for s in doc['segments'] if s['kind'] == 'weld'}
    for ev in doc['events']:
        seg = by_id.get(ev['after_segment'])
        expected = 'approach' if ev['type'] == 'WeldStart' else 'weld'
        if seg is None or seg['kind'] != expected or seg['seam_id'] != ev['seam_id']:
            problems.append(f'{ev["event_id"]} {ev["type"]} {ev["seam_id"]} not after the '
                            f'{expected} segment of its seam')
    for sid in seams:
        types = [e['type'] for e in doc['events'] if e['seam_id'] == sid]
        if types != ['WeldStart', 'WeldEnd']:
            problems.append(f'{sid}: events {types}')
    return not problems, '; '.join(problems) or (
        f'{len(doc["events"])} events, each at a segment boundary (segments end at rest)')


def _dense(q: np.ndarray) -> np.ndarray:
    out = [q[0]]
    for a, b in zip(q, q[1:]):
        n = int(np.ceil(np.max(np.abs(b - a)) / COLLISION_STEP_RAD))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def check_collisions(doc: Dict, client: MoveItClient) -> Tuple[bool, str]:
    problems, count = [], 0
    for s in doc['segments']:
        q = _dense(np.array([p['q_rad'] for p in s['points']]))
        for qi in q:
            count += 1
            valid, contacts = client.state_valid(qi)
            if not valid:
                problems.append(f'{s["segment_id"]} ({s["kind"]} {s["seam_id"] or ""}): '
                                f'{", ".join(contacts) or "invalid state"}')
                break
    return not problems, '; '.join(problems) or f'{count} states collision-free'


def verify_program(job: JobPaths, rev: str, params: ChainParams, client: MoveItClient,
                   replayer: Optional[Replayer]) -> Dict:
    """Run all checks, write programs/<rev>/verify/report.json, return it."""
    program_path = job.program_json(rev)
    checks: Dict[str, Dict] = {}
    try:
        doc = jsonio.read_json(program_path, 'program', base_dir=job.dir)
        checks['inputs'] = {'ok': True, 'detail': 'program and its inputs unchanged'}
    except jsonio.StaleInputError as exc:
        doc = jsonio.read_json(program_path, 'program')
        checks['inputs'] = {'ok': False, 'detail': str(exc)}
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    cal = load_active(job.repo)
    load_part_into_scene(client, job, part, cal)
    for name, fn in (('continuity', check_continuity), ('events_at_rest', check_events)):
        ok, detail = fn(doc)
        checks[name] = {'ok': ok, 'detail': detail}
    ok, detail = check_limits(doc, params)
    checks['limits'] = {'ok': ok, 'detail': detail}
    ok, detail = check_collisions(doc, client)
    checks['collision'] = {'ok': ok, 'detail': detail}

    replay = {'executed': False, 'segments_ok': 0, 'segments_total': len(doc['segments']),
              'max_tracking_error_rad': None, 'measured_time_s': None}
    pre_ok = all(c['ok'] for c in checks.values())
    if replayer is None:
        checks['replay'] = {'ok': False, 'detail': 'replay on mock hardware not run'}
    elif not pre_ok:
        checks['replay'] = {'ok': False, 'detail': 'not replayed: static checks failed'}
    else:
        results = replayer.run_program(doc['segments'], params.replay_speed_scale,
                                       params.start_state_tolerance_rad,
                                       params.max_tracking_error_rad)
        good = [r for r in results if r.ok]
        replay.update({'executed': True, 'segments_ok': len(good),
                       'max_tracking_error_rad': float(max(r.end_error_rad for r in results)),
                       'measured_time_s': float(sum(r.duration_s for r in results))})
        failed = [r for r in results if not r.ok]
        checks['replay'] = {
            'ok': not failed and len(results) == len(doc['segments']),
            'detail': (f'{len(good)}/{len(doc["segments"])} segments on mock hardware at '
                       f'{params.replay_speed_scale:g}x speed' +
                       (f'; {failed[0].segment_id}: {failed[0].detail}' if failed else ''))}
    path = jsonio.read_json(job.path_json, 'path', check=False)
    lengths = {s['seam_id']: s['weld_length_m'] for s in path['seams']}
    seams = [{'seam_id': s['seam_id'], 'weld_time_s': s['duration_s'],
              'length_m': float(lengths.get(s['seam_id'], 0.0))}
             for s in doc['segments'] if s['kind'] == 'weld']
    report = jsonio.make_header(CREATED_BY, [program_path], job.dir)
    report.update({
        'program_rev': rev, 'program_sha256': sha256_file(program_path), 'checks': checks,
        'replay': replay, 'cycle_time_s': doc['cycle_time_s'], 'seams': seams,
        'result': 'PASS' if all(c['ok'] for c in checks.values()) else 'FAIL'})
    jsonio.write_json(job.report_json(rev), report, 'verify_report')
    for name, c in checks.items():
        (LOG.info if c['ok'] else LOG.error)('%s: %s', name, c['detail'])
    LOG.info('A71 %s: %s (report %s)', rev, report['result'], job.report_json(rev))
    return report
