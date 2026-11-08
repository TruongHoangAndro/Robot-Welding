#!/usr/bin/env python3
"""Rebuild every golden-part data set from the cell STEP alone (G1.22, close-out C1, Truong 2026-11-08).

Runs INSIDE ``awm_dev`` (ROS python with the workspace overlay, the FastAPI venv and ``AWM_OCC_PYTHON``)::

    scripts/dx "python3 scripts/golden_rebuild.py --cell-step <cell STEP> --out <repository root> [--force]"

The only external input is the cell STEP (``--cell-step`` or ``AWM_CELL_STEP``). Every pick a user makes in the
browser (roles, edge ids, component names, speeds, the CP-62 reason) comes from one data file, by default
``src/awm_bringup/test/data/golden_564082.yaml`` (test data, CLAUDE.md design rule 2). The chain is driven through the
real backend: an in-process ``awm_api`` app (FastAPI ``TestClient``) with its default CLI commands, so the cell / job
imports, the CP-60 gate, the CP-62 override, the plan and program runs and their promotion rules are exactly the
browser's. Two files a user places by hand are made with ``awm_geometry make-job-step`` from the same cell STEP: the
head STEP (robot + mount + LightWELD, for the CP-61 TCP teach) and the job STEP (CP-60 rotating set + part).

Steps: head STEP + head map → S10 cell import → cell roles (duplicate welding tool excluded, OI-23) → positioner axis
pick → TCP teach (checked against the expected tool0 TCP) → save cell model → job STEP → S1 job import → job roles →
job axis → cross-check (expected reds only) → CP-62 override (Admin) → J1 / J2 seams → plan → program → S4 files
→ ``golden_summary.json``.

Rules: refuse on any mismatch (a picked edge that is not a circle of the expected radius, a TCP off by more than the
tolerance, an unexpected cross-check result, a rejected or partial seam): no fallback. Idempotent: an output folder
whose summary has the same fingerprint (cell STEP, data file, this script and the engineering package sources) is
left as it is; otherwise it is rebuilt from scratch. Only a folder made by this script is ever cleared. Simulation
data only: nothing here talks to hardware.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import hashlib
import json
import math
import os
import secrets
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence

WS = Path(__file__).resolve().parent.parent
DEFAULT_DATA = WS / 'src' / 'awm_bringup' / 'test' / 'data' / 'golden_564082.yaml'
OUT_ENV = 'AWM_GOLDEN_OUT'
CELL_ENV = 'AWM_CELL_STEP'
DEFAULT_OUT = WS / 'build' / 'golden_564082'
SCHEMA = 'awm_golden_rebuild/0.1'
SUMMARY = 'golden_summary.json'
MARKER = '.golden_rebuild'
HEAD_STEP, HEAD_MAP = 'head.step', 'cad_map_head.yaml'
S4_DIR = 's4_golden'
#: Packages whose sources invalidate a cached rebuild: these roots plus their workspace dependency closure
#: (package.xml depend / build / exec tags, tests excluded).
FINGERPRINT_ROOTS = ('awm_api', 'awm_path', 'awm_program')
_DEP_TAGS = ('depend', 'build_depend', 'build_export_depend', 'exec_depend')
_SKIP_DIRS = {'test', 'tests', '__pycache__', 'node_modules', '.pytest_cache'}
SKIP_REASON = f'{CELL_ENV} not set'


class GoldenError(RuntimeError):
    """The CAD, a CLI result or a gate does not match the golden data: the rebuild stops."""


# --- data file ----------------------------------------------------------------------------------------------------

_REQUIRED = {
    'users': ('admin', 'engineer'),
    'tolerances': ('radius_mm', 'tcp_mm', 'joint_length_mm'),
    'cell': ('up_axis', 'roles', 'positioner_axis'),
    'head': ('component', 'up_axis', 'map', 'tcp_teach', 'expected_tcp_tool0_mm'),
    'job': ('part_no', 'part_rev', 'up_axis', 'components', 'roles', 'positioner_axis', 'cross_check_red',
            'override_reason', 'seams', 'seam_defaults', 'travel_speed_mm_s'),
}


def load_data(path: Path) -> Dict[str, Any]:
    """Read and check the golden data file; every missing key is a refusal."""
    import yaml
    path = Path(path)
    if not path.is_file():
        raise GoldenError(f'golden data file not found: {path}')
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or doc.get('schema') != SCHEMA:
        raise GoldenError(f'{path}: schema must be {SCHEMA!r}')
    missing = [f'{block}.{key}' for block, keys in _REQUIRED.items() for key in keys
               if not isinstance(doc.get(block), dict) or key not in doc[block]]
    if missing:
        raise GoldenError(f'{path}: missing keys {missing}')
    ids = [s.get('seam_id') for s in doc['job']['seams']]
    if not ids or len(set(ids)) != len(ids):
        raise GoldenError(f'{path}: job.seams needs unique seam ids, got {ids}')
    for s in doc['job']['seams']:
        for key in ('seam_id', 'label', 'joint_edge_id', 'length_mm', 'coaxial'):
            if key not in s:
                raise GoldenError(f'{path}: seam {s.get("seam_id")} misses {key}')
    if not float(doc['job']['travel_speed_mm_s']) > 0.0:
        raise GoldenError(f'{path}: job.travel_speed_mm_s must be > 0')
    return doc


# --- checks (pure) ------------------------------------------------------------------------------------------------

def check_circle(pick: Dict[str, Any], edge_id: str, radius_mm: float, tol_mm: float, what: str) -> None:
    """A circle pick (``v2/axis_pick`` record) must be the expected edge with the expected radius."""
    if pick.get('edge_id') != edge_id:
        raise GoldenError(f'{what}: picked edge {pick.get("edge_id")!r}, expected {edge_id!r}')
    got = pick.get('radius_mm')
    if not isinstance(got, (int, float)) or abs(float(got) - float(radius_mm)) > tol_mm:
        raise GoldenError(f'{what}: edge {edge_id} has radius {got} mm, expected {radius_mm} ± {tol_mm} mm')


def check_tcp(head: Dict[str, Any], teach: Dict[str, Any], expected_mm: Sequence[float], tol_mm: float,
              radius_tol_mm: float) -> List[float]:
    """Taught TCP in tool0 (``head.yaml`` ``tool0_T_tcp_mm.xyz``) within ``tol_mm`` (distance) of the expected point;
    the teach record (``estimates.tcp``) must be the expected component / edge with the expected mouth radius.
    Returns the TCP."""
    try:
        xyz = [float(v) for v in head['tool0_T_tcp_mm']['xyz']]
    except (KeyError, TypeError, ValueError) as exc:
        raise GoldenError(f'head.yaml has no tool0_T_tcp_mm.xyz: {exc}') from exc
    rec = (head.get('estimates') or {}).get('tcp') or {}
    if rec.get('method') != 'teach_ui' or rec.get('edge_id') != teach['edge_id'] \
            or rec.get('component') != teach['component']:
        raise GoldenError(f'head.yaml TCP is not the teach of {teach["component"]} {teach["edge_id"]}: method '
                          f'{rec.get("method")!r}, {rec.get("component")!r} {rec.get("edge_id")!r}')
    got_r = rec.get('mouth_radius_mm')
    if not isinstance(got_r, (int, float)) or abs(float(got_r) - float(teach['radius_mm'])) > radius_tol_mm:
        raise GoldenError(f'taught nozzle circle {teach["edge_id"]} has radius {got_r} mm, expected '
                          f'{teach["radius_mm"]} ± {radius_tol_mm} mm')
    dist = math.dist(xyz, [float(v) for v in expected_mm])
    if dist > tol_mm:
        raise GoldenError(f'taught TCP {[round(v, 4) for v in xyz]} mm in tool0 is {dist:.4f} mm from the expected '
                          f'{list(expected_mm)} (tolerance {tol_mm} mm)')
    return xyz


def resolve_seams(topology: Dict[str, Any], job: Dict[str, Any], length_tol_mm: float) -> List[Dict[str, Any]]:
    """S2 picks from the joint ids of the data file: each joint must exist, be closed, have the expected length and
    join two solids of a ``part`` component (paths from the job roles). The seam edge is the joint's ``edge_a``."""
    joints = {j['joint_edge_id']: j for j in topology.get('joints') or []}
    paths = {s['solid_id']: s.get('component_path') or '' for s in topology.get('solids') or []}
    parts = [a['path'] for a in job['roles'] if a['role'] == 'part']
    picks = []
    for k, s in enumerate(job['seams']):
        j = joints.get(s['joint_edge_id'])
        if j is None:
            raise GoldenError(f'{s["label"]}: joint {s["joint_edge_id"]} not in the job topology '
                              f'({len(joints)} joints)')
        if not j.get('closed'):
            raise GoldenError(f'{s["label"]}: joint {s["joint_edge_id"]} is not closed')
        if abs(float(j['length_mm']) - float(s['length_mm'])) > length_tol_mm:
            raise GoldenError(f'{s["label"]}: joint {s["joint_edge_id"]} length {j["length_mm"]} mm, expected '
                              f'{s["length_mm"]} ± {length_tol_mm} mm')
        for side in ('solid_a', 'solid_b'):
            owner = paths.get(j.get(side), '')
            if not any(owner == p or owner.startswith(p + ' / ') for p in parts):
                raise GoldenError(f'{s["label"]}: joint {s["joint_edge_id"]} {side} belongs to {owner!r}, not to a '
                                  f'part component {parts}')
        pick = dict(job['seam_defaults'])
        pick.update({'seam_id': s['seam_id'], 'edges': [j['edge_a']], 'sequence': k + 1})
        picks.append(pick)
    return picks


def check_cross_check(check: Dict[str, Any], expected_red: Iterable[int]) -> List[Dict[str, Any]]:
    """The reds of ``cad_check.json`` must be exactly the expected check ids; returns the red checks."""
    reds = [c for c in check.get('checks') or [] if c.get('status') == 'red']
    got, want = sorted(int(c['id']) for c in reds), sorted(int(i) for i in expected_red)
    if got != want:
        raise GoldenError(f'cross-check reds {got}, expected {want}: ' + '; '.join(
            f'{c["id"]} {c.get("name")}: {(c.get("note") or "")[:200]}' for c in reds))
    if want and check.get('status') != 'red':
        raise GoldenError(f'cross-check status {check.get("status")!r} with red checks {got}')
    return reds


def check_coaxial(seam_axis: Dict[str, Any], job: Dict[str, Any]) -> Dict[str, bool]:
    """``seam_axis.json`` coaxial flag of each seam against the data file (J1 on the axis, J2 off it)."""
    got = {}
    for s in seam_axis.get('seams') or []:
        flag = (s.get('coaxial') or {}).get('is_coaxial') if isinstance(s.get('coaxial'), dict) else None
        if not isinstance(flag, bool):
            raise GoldenError(f'seam_axis.json {s.get("seam_id")}: no coaxial.is_coaxial flag')
        got[s['seam_id']] = flag
    for s in job['seams']:
        if s['seam_id'] not in got:
            raise GoldenError(f'{s["label"]}: {s["seam_id"]} missing in seam_axis.json')
        if got[s['seam_id']] != bool(s['coaxial']):
            raise GoldenError(f'{s["label"]}: coaxial = {got[s["seam_id"]]}, expected {s["coaxial"]}')
    return got


def check_path(path: Dict[str, Any], job: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every seam planned to 100 % (all points ok, nothing rejected); returns one record per seam."""
    if path.get('rejected'):
        raise GoldenError('planner rejected: ' + '; '.join(
            f'{r.get("seam_id")}: {r.get("reason")}' for r in path['rejected']))
    by_id = {s['seam_id']: s for s in path.get('seams') or []}
    out = []
    for spec in job['seams']:
        s = by_id.get(spec['seam_id'])
        if s is None:
            raise GoldenError(f'{spec["label"]}: {spec["seam_id"]} not in path.json')
        pts = s.get('points') or []
        ok = sum(1 for p in pts if p.get('status') == 'ok')
        if not pts or ok != len(pts):
            raise GoldenError(f'{spec["label"]}: {ok} / {len(pts)} points ok (a seam is 100 % or rejected)')
        plan = s.get('planning') or {}
        out.append({'seam_id': spec['seam_id'], 'label': spec['label'], 'joint_edge_id': spec['joint_edge_id'],
                    'mode': s.get('mode'), 'coverage_pct': 100.0, 'points': len(pts),
                    'travel_speed_mm_s': s.get('travel_speed_mm_s'),
                    'max_travel_speed_mm_s': plan.get('max_travel_speed_mm_s'),
                    'theta_travel_deg': plan.get('theta_travel_deg'),
                    'wrist3_travel_deg': plan.get('wrist3_travel_deg'),
                    'fk_max_error_mm': plan.get('fk_max_error_mm'), 'config_label': s.get('config_label')})
    return out


def robot_frame_transform(tree: List[Dict[str, Any]], name: str) -> List[float]:
    """``make-job-step --transform`` values (mm, deg URDF rpy) that put the named top component at the file origin:
    the head STEP is written in the robot's own frame (like a Fusion export of the robot assembly), read from the
    cell component tree of the import, never typed. Also keeps the head export away from the large cell
    coordinates (its hull containment check failed there, see the G1.22 report)."""
    import numpy as np

    from awm_core import frames, units
    hits = [n for n in tree if n.get('parent') is not None and n.get('instance_name') == name]
    if len(hits) != 1:
        raise GoldenError(f'head component {name!r} matches {len(hits)} components of the cell (expected 1)')
    inv = frames.invert(np.asarray(hits[0]['placement_mm'], dtype=float))
    return [float(v) for v in inv[:3, 3]] + [float(units.rad_to_deg(v)) for v in frames.matrix_to_rpy(inv[:3, :3])]


def cli_numbers(values: Sequence[float]) -> List[str]:
    """Fixed-point text for argparse (a value like ``-1e-14`` would be read as an option)."""
    return [f'{float(v):.12f}' for v in values]


def head_map(head: Dict[str, Any], sha256: str) -> Dict[str, Any]:
    """``cad_map_head.yaml`` for the head STEP cut from the cell (names and rules from the data file)."""
    m = head['map']
    return {'schema': 'cad_map/0.1',
            'source': {'file': HEAD_STEP, 'sha256': sha256, 'unit': 'mm', 'up_axis': head['up_axis']},
            'match': 'instance_name', 'default': 'deny',
            'roles': m['roles'], 'head_links': m['head_links'], 'frames': m['frames']}


# --- fingerprint / output folder ----------------------------------------------------------------------------------

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open('rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def fingerprint_packages(ws: Path, roots: Sequence[str] = FINGERPRINT_ROOTS) -> List[str]:
    """Workspace packages in the dependency closure of ``roots`` (sorted), read from ``src/*/package.xml``."""
    import xml.etree.ElementTree as ET
    deps: Dict[str, List[str]] = {}
    for xml in sorted((Path(ws) / 'src').glob('*/package.xml')):
        root = ET.parse(xml).getroot()
        name = (root.findtext('name') or xml.parent.name).strip()
        deps[name] = [(e.text or '').strip() for tag in _DEP_TAGS for e in root.findall(tag)]
    seen: set = set()
    todo = [r for r in roots]
    while todo:
        name = todo.pop()
        if name in seen or name not in deps:
            continue
        seen.add(name)
        todo.extend(deps[name])
    missing = [r for r in roots if r not in deps]
    if missing:
        raise GoldenError(f'fingerprint roots {missing} have no package.xml under {ws}/src')
    return sorted(seen)


def environment(ws: Path = WS) -> Dict[str, str]:
    """Tool versions that change the rebuild result: Dockerfile, OCC interpreter + version, ur_analytic_ik,
    ur_description. A missing piece is a refusal (no fallback)."""
    import importlib.metadata
    import xml.etree.ElementTree as ET

    from awm_core import occ_bridge
    try:
        occ = occ_bridge.interpreter_command(None)
        proc = subprocess.run(occ + ['-c', 'import OCC; print(OCC.VERSION)'], capture_output=True, text=True,
                              env=occ_bridge.geometry_env(), timeout=120)
        if proc.returncode != 0:
            raise GoldenError(f'OCC version: {proc.stderr[-500:]}')
        from ament_index_python.packages import get_package_share_directory
        ur = ET.parse(Path(get_package_share_directory('ur_description')) / 'package.xml').getroot()
        return {'dockerfile_sha256': sha256_file(Path(ws) / 'docker' / 'Dockerfile'),
                'occ_python': ' '.join(occ), 'occ_version': proc.stdout.strip(),
                'ur_analytic_ik': importlib.metadata.version('ur_analytic_ik'),
                'ur_description': (ur.findtext('version') or '').strip()}
    except (occ_bridge.GeometryCliError, importlib.metadata.PackageNotFoundError, LookupError, OSError) as exc:
        raise GoldenError(f'environment digest: {exc}') from exc


def source_digest(ws: Path, packages: Optional[Sequence[str]] = None) -> str:
    """SHA-256 over the relative path and content of every file of the engineering package sources (no tests)."""
    h = hashlib.sha256()
    for pkg in (fingerprint_packages(ws) if packages is None else packages):
        root = ws / 'src' / pkg
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = sorted(d for d in dirnames if d not in _SKIP_DIRS and not d.startswith('.'))
            for name in sorted(filenames):
                if name.endswith(('.pyc', '.pyo')):
                    continue
                p = Path(dirpath) / name
                h.update(p.relative_to(ws).as_posix().encode() + b'\0')
                h.update(p.read_bytes() + b'\0')
    return h.hexdigest()


def fingerprint(cell_step: Path, data: Path, ws: Path = WS) -> Dict[str, str]:
    env = json.dumps(environment(ws), sort_keys=True).encode()
    return {'cell_step_sha256': sha256_file(cell_step), 'data_sha256': sha256_file(data),
            'script_sha256': sha256_file(Path(__file__)), 'sources_sha256': source_digest(ws),
            'environment_sha256': hashlib.sha256(env).hexdigest()}


def read_summary(out: Path) -> Optional[Dict[str, Any]]:
    try:
        return json.loads((Path(out) / SUMMARY).read_text())
    except (OSError, ValueError):
        return None


def up_to_date(out: Path, fp: Dict[str, str]) -> bool:
    doc = read_summary(out)
    return bool(doc) and doc.get('fingerprint') == fp and doc.get('status') == 'ok'


def prepare_out(out: Path) -> None:
    """Empty the output folder for a rebuild. A non-empty folder without the rebuild marker is refused, never
    cleared (it could be somebody's repository)."""
    out = Path(out)
    if out.exists():
        if not out.is_dir():
            raise GoldenError(f'{out} exists and is not a folder')
        if any(out.iterdir()) and not (out / MARKER).is_file():
            raise GoldenError(f'{out} is not empty and was not made by golden_rebuild ({MARKER} missing): refused')
        shutil.rmtree(out)
    out.mkdir(parents=True)
    (out / MARKER).write_text('made by scripts/golden_rebuild.py; rebuilt from scratch when its inputs change\n')


@contextlib.contextmanager
def out_lock(out: Path):
    """One rebuild per output folder at a time (colcon may test packages in parallel)."""
    lock = Path(str(out) + '.lock')
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('w') as fh:
        fcntl.flock(fh, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(fh, fcntl.LOCK_UN)


# --- the chain through awm_api ------------------------------------------------------------------------------------

class Chain:
    """The browser chain against an in-process awm_api (real CLIs, real gates)."""

    def __init__(self, client, log: Callable[[str], None]):
        self.c, self.log, self.times = client, log, {}

    def login(self, user: str, password: str) -> Dict[str, str]:
        r = self.c.post('/api/v1/auth/login', json={'username': user, 'password': password})
        self._ok(r, f'login {user}')
        return {'Authorization': f'Bearer {r.json()["token"]}'}

    def _ok(self, r, what: str, codes=(200, 201, 202, 204)):
        if r.status_code not in codes:
            raise GoldenError(f'{what}: HTTP {r.status_code} {r.text[:1500]}')
        return r.json() if r.content else None

    def call(self, method: str, path: str, hdr, what: str, **kw):
        return self._ok(self.c.request(method, '/api/v1' + path, headers=hdr, **kw), what)

    def run(self, method: str, path: str, hdr, what: str, timeout_s: float = 3600.0, **kw) -> Dict[str, Any]:
        """Start a run and wait for it; a failed run stops the rebuild with its problem and log tail."""
        t0 = time.monotonic()
        doc = self.call(method, path, hdr, what, **kw)
        run_id = doc['run_id']
        while True:
            r = self.c.get(f'/api/v1/runs/{run_id}', headers=hdr)
            run = self._ok(r, f'{what} status')
            if run['state'] not in ('queued', 'running'):
                break
            if time.monotonic() - t0 > timeout_s:
                self.c.delete(f'/api/v1/runs/{run_id}', headers=hdr)
                raise GoldenError(f'{what}: no result after {timeout_s:.0f} s (cancelled)')
            time.sleep(1.0)
        dt = time.monotonic() - t0
        self.times[what] = round(dt, 1)
        self.log(f'  {what}: {run["state"]} in {dt:.0f} s')
        if run['state'] != 'succeeded':
            raise GoldenError(f'{what}: run {run["state"]}: {json.dumps(run.get("problem"))[:1500]}\n  '
                              + '\n  '.join(run.get('log_tail', [])[-15:]))
        return run


def _occ(args: List[str], what: str, log: Callable[[str], None]) -> float:
    """``awm_geometry.cli`` in the OCC interpreter (ADR-15), the way the backend adapters call it."""
    from awm_core import occ_bridge
    argv = occ_bridge.interpreter_command(None) + ['-m', 'awm_geometry.cli'] + args
    t0 = time.monotonic()
    proc = subprocess.run(argv, env=occ_bridge.geometry_env(), capture_output=True, text=True)
    dt = time.monotonic() - t0
    if proc.returncode != 0:
        raise GoldenError(f'{what}: awm_geometry {args[0]} exit {proc.returncode}\n{proc.stderr[-2000:]}')
    log(f'  {what}: ok in {dt:.0f} s')
    return dt


def _users_file(path: Path, data: Dict[str, Any]) -> Dict[str, str]:
    """Throwaway Admin + Engineer accounts with random passwords (kept in memory only)."""
    from awm_api.auth import hash_password
    pw = {data['users']['admin']: secrets.token_urlsafe(18), data['users']['engineer']: secrets.token_urlsafe(18)}
    roles = {data['users']['admin']: 'Admin', data['users']['engineer']: 'Engineer'}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'users': [{'username': u, 'role': roles[u], 'password': hash_password(p)}
                                          for u, p in pw.items()]}))
    return pw


def _read(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError) as exc:
        raise GoldenError(f'{path}: {exc}') from exc


def rebuild(cell_step: Path, out: Path, data_path: Path, fp: Dict[str, str],
            log: Callable[[str], None] = print) -> Dict[str, Any]:
    """Build the golden repository at ``out`` (emptied first) and return the summary."""
    import yaml
    from fastapi.testclient import TestClient

    from awm_api.app import create_app
    from awm_api.settings import Settings
    from awm_geometry import miniyaml

    t_all = time.monotonic()
    data = load_data(data_path)
    tol = data['tolerances']
    cell, head, job = data['cell'], data['head'], data['job']
    prepare_out(out)
    pw = _users_file(out / 'secrets' / 'awm_api_users.json', data)
    cad = out / 'cell' / 'cad'
    cad.mkdir(parents=True)
    times: Dict[str, float] = {}

    inputs = out / 'inputs'
    inputs.mkdir()
    job_step = inputs / f'{job["part_no"]}_job.step'

    settings = Settings(repo_root=out)
    admin, eng = data['users']['admin'], data['users']['engineer']
    with TestClient(create_app(settings)) as client:
        ch = Chain(client, log)
        adm = ch.login(admin, pw[admin])
        eh = ch.login(eng, pw[eng])

        log('S10 cell import')
        with Path(cell_step).open('rb') as fh:
            ch.run('POST', '/cell/model', adm, 'cell import', files={'step': (Path(cell_step).name, fh,
                                                                              'application/step')},
                   data={'up_axis': cell['up_axis']})
        log('head STEP: robot + head cut from the cell STEP in the robot frame (CP-61 needs the flange and the '
            'mount in one file)')
        move = robot_frame_transform(_read(cad / 'cell_components.json')['components'], head['component'])
        times['make head STEP'] = round(_occ(['make-job-step', '--step', str(cell_step), '--components',
                                              head['component'], '--out', str(cad / HEAD_STEP), '--transform',
                                              *cli_numbers(move)], 'make head STEP', log), 1)
        miniyaml.dump(head_map(head, sha256_file(cad / HEAD_STEP)), cad / HEAD_MAP,
                      header='Head map written by scripts/golden_rebuild.py from the golden data file (G1.22).')
        ch.call('PUT', '/cell/model/roles', adm, 'cell roles', json={
            'assignments': cell['roles'], 'collision': cell.get('collision'),
            'contact_tolerance_mm': cell.get('contact_tolerance_mm')})
        ax = cell['positioner_axis']
        ch.run('PUT', '/cell/model/axis', adm, 'cell axis pick', json={
            'component_path': ax['component_path'], 'edge_id': ax['edge_id'], 'reverse': bool(ax.get('reverse'))})
        cell_pick = _read(cad / 'axis_pick.json')
        check_circle(cell_pick, ax['edge_id'], ax['radius_mm'], tol['radius_mm'], 'cell positioner axis')
        if cell_pick.get('component_path') != ax['component_path']:
            raise GoldenError(f'cell axis pick on {cell_pick.get("component_path")!r}, expected '
                              f'{ax["component_path"]!r}')

        tt = head['tcp_teach']
        ch.run('PUT', '/cell/head/tcp', adm, 'TCP teach', json={
            'component': tt['component'], 'edge_id': tt['edge_id'], 'reverse': bool(tt.get('reverse')),
            'stickout_mm': float(tt.get('stickout_mm', 0.0))})
        head_doc = yaml.safe_load((out / 'cell' / 'model' / 'head' / 'head.yaml').read_text())
        tcp = check_tcp(head_doc, tt, head['expected_tcp_tool0_mm'], tol['tcp_mm'], tol['radius_mm'])
        ch.run('POST', '/cell/model/save', adm, 'save cell model', json={})
        model = out / 'cell' / 'model'
        manifest = _read(model / 'scene_manifest.json')
        excluded = [a['path'] for a in cell['roles'] if a['role'] == 'exclude']
        status = ch.call('GET', '/cell/model', adm, 'cell status')
        if not status['model']['saved'] or status['roles']['state'] != 'ok' or status['axis']['state'] != 'ok':
            raise GoldenError(f'cell model not complete after save: {json.dumps(status)[:800]}')

        log('job STEP (CP-60 make-job-step) and S1 import')
        times['make job STEP'] = round(_occ(['make-job-step', '--step', str(cell_step), '--components',
                                             *job['components'], '--out', str(job_step)], 'make job STEP', log), 1)
        job_id = f'{job["part_no"]}_{job["part_rev"]}'
        with job_step.open('rb') as fh:
            ch.run('POST', '/jobs', eh, 'job import', files={'step': (job_step.name, fh, 'application/step')},
                   data={'part_no': str(job['part_no']), 'part_rev': str(job['part_rev']), 'up_axis': job['up_axis'],
                         'assembly': 'true'})
        jdir = out / 'jobs' / job_id
        ch.call('PUT', f'/jobs/{job_id}/roles', eh, 'job roles', json={'assignments': job['roles']})
        jax = job['positioner_axis']
        ch.run('PUT', f'/jobs/{job_id}/axis', eh, 'job axis pick', json={
            'edge_id': jax['edge_id'], 'reverse': bool(jax.get('reverse'))})
        check_circle(_read(jdir / 'axis_pick.json'), jax['edge_id'], jax['radius_mm'], tol['radius_mm'],
                     'job positioner axis')
        ch.run('POST', f'/jobs/{job_id}/check', eh, 'cross-check', json={})
        cc = ch.call('GET', f'/jobs/{job_id}/check', eh, 'cross-check result')
        if cc['state'] != 'ok':
            raise GoldenError(f'cross-check state {cc["state"]!r} right after the check')
        reds = check_cross_check(cc['check'], job['cross_check_red'])
        override = None
        if reds:
            override = ch.call('POST', f'/jobs/{job_id}/check/override', adm, 'CP-62 override',
                               json={'reason': ' '.join(str(job['override_reason']).split())})

        log('S2 seams, plan, program')
        picks = resolve_seams(_read(jdir / 'cache' / 'view' / 'topology.json'), job, tol['joint_length_mm'])
        ch.call('PUT', f'/jobs/{job_id}/seams', eh, 'seams', json={'seams': picks})
        coaxial = check_coaxial(_read(jdir / 'seam_axis.json'), job)
        speed = float(job['travel_speed_mm_s'])
        ch.run('POST', f'/jobs/{job_id}/plan', eh, 'plan', json={'travel_speed_mm_s': speed, 'collision': 'off'})
        path = _read(jdir / 'path.json')
        seams = check_path(path, job)
        if override is not None and (path.get('cad_check_override') or {}).get('by') != admin:
            raise GoldenError(f'path.json does not carry the CP-62 override of {admin}: '
                              f'{path.get("cad_check_override")}')
        run = ch.run('POST', f'/jobs/{job_id}/program', eh, 'program', json={})
        sim = ch.call('GET', f'/jobs/{job_id}/simulation', eh, 'simulation')
        prog_rel = next(iter(run.get('outputs') or {}), None)
        if not prog_rel:
            raise GoldenError(f'program run has no output: {run}')
        program = _read(jdir / prog_rel)
        if override is not None and (program.get('cad_check_override') or {}).get('by') != admin:
            raise GoldenError('program.json does not carry the CP-62 override')
        if program.get('collision_checked') is not False:
            raise GoldenError('program.json must record collision_checked false (Goal 1 kinematic only)')

        s4 = out / S4_DIR
        s4.mkdir()
        (s4 / 'robot_description.json').write_text(json.dumps(
            ch.call('GET', '/cell/robot_description', eh, 'robot description')))
        shutil.copyfile(jdir / 'path.json', s4 / 'path.json')
        shutil.copyfile(jdir / 'placement.json', s4 / 'placement.json')
        times.update(ch.times)

    welds = {w['seam_id']: w for w in program['generator']['weld_segments']}
    for s in seams:
        w = welds.get(s['seam_id'], {})
        s.update(coaxial=coaxial[s['seam_id']], weld_segment=w.get('segment_id'),
                 weld_duration_s=w.get('duration_s'), program_fk_max_error_mm=w.get('fk_max_error_mm'))
    summary = {
        'schema': 'awm_golden_summary/0.1', 'status': 'ok', 'fingerprint': fp,
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'repo': str(out), 'data_file': os.path.relpath(data_path, WS), 'job_id': job_id,
        'joints': {s['label']: s['joint_edge_id'] for s in job['seams']},
        'seam_edges': {p['seam_id']: p['edges'][0] for p in picks},
        'cell': {'positioner_axis': {k: cell_pick.get(k) for k in ('component_path', 'edge_id', 'radius_mm',
                                                                    'center_file_mm', 'axis_file')},
                 'excluded': excluded, 'scene_objects': len(manifest.get('objects') or []),
                 'scene_object_ids': [o.get('object_id') for o in manifest.get('objects') or []]},
        'head': {'tcp_tool0_mm': tcp, 'expected_tcp_tool0_mm': head['expected_tcp_tool0_mm'],
                 'tcp_error_mm': math.dist(tcp, head['expected_tcp_tool0_mm']),
                 'teach': {k: tt.get(k) for k in ('component', 'edge_id', 'radius_mm', 'stickout_mm')},
                 'quat_xyzw': head_doc['tool0_T_tcp_mm'].get('quat_xyzw')},
        'cross_check': {'status': cc['check'].get('status'), 'phi_deg': cc['check'].get('phi_deg'),
                        'checks': [{k: c.get(k) for k in ('id', 'name', 'status', 'note')}
                                   for c in cc['check'].get('checks') or []]},
        'override': None if override is None else {k: override.get(k) for k in (
            'by', 'role', 'created_at', 'reason', 'scope', 'check_id', 'cad_check_sha256', 'findings')},
        'travel_speed_mm_s': speed, 'seams': seams,
        'program': {'program_rev': program.get('program_rev'), 'file': f'jobs/{job_id}/{prog_rel}',
                    'cycle_time_s': program.get('cycle_time_s'), 'segments': len(program.get('segments') or []),
                    'segment_kinds': [s.get('kind') for s in program.get('segments') or []],
                    'events': len(program.get('events') or []), 'collision_checked': program['collision_checked'],
                    'cad_check_override': program.get('cad_check_override'),
                    'simulation_cycle_time_s': (sim.get('program') or {}).get('cycle_time_s')},
        's4_dir': S4_DIR, 'times_s': times, 'runtime_s': round(time.monotonic() - t_all, 1),
    }
    (out / SUMMARY).write_text(json.dumps(summary, indent=1, default=float))
    log(f'golden rebuild ok in {summary["runtime_s"]:.0f} s: {out / SUMMARY}')
    return summary


def ensure_golden(cell_step: Path, out: Optional[Path] = None, data: Optional[Path] = None, force: bool = False,
                  log: Callable[[str], None] = print) -> Dict[str, Any]:
    """Summary of an up-to-date golden repository at ``out``, rebuilt first when needed (locked, idempotent)."""
    cell_step = Path(cell_step).resolve()
    if not cell_step.is_file():
        raise GoldenError(f'cell STEP not found: {cell_step}')
    out = Path(out or os.environ.get(OUT_ENV, '').strip() or DEFAULT_OUT).resolve()
    data = Path(data or DEFAULT_DATA).resolve()
    with out_lock(out):
        fp = fingerprint(cell_step, data)
        if not force and up_to_date(out, fp):
            log(f'golden repository up to date: {out}')
            return read_summary(out)
        return rebuild(cell_step, out, data, fp, log)


def pytest_golden() -> Dict[str, Any]:
    """For the session fixtures of the golden tests: skip with ``AWM_CELL_STEP not set``, else the summary."""
    import pytest
    step = os.environ.get(CELL_ENV, '').strip()
    if not step:
        pytest.skip(SKIP_REASON)
    return ensure_golden(Path(step), log=lambda m: print('[golden_rebuild]', m, file=sys.stderr, flush=True))


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--cell-step', default=os.environ.get(CELL_ENV), help=f'cell STEP (else {CELL_ENV})')
    ap.add_argument('--out', default=None, help=f'golden repository root (else {OUT_ENV}, else build/golden_564082)')
    ap.add_argument('--data', default=None, help='golden data file (default src/awm_bringup/test/data/'
                                                 'golden_564082.yaml)')
    ap.add_argument('--force', action='store_true', help='rebuild even when the summary is up to date')
    args = ap.parse_args(argv)
    if not args.cell_step:
        print(f'golden_rebuild: --cell-step or {CELL_ENV} required', file=sys.stderr)
        return 2
    t0 = time.monotonic()
    try:
        summary = ensure_golden(Path(args.cell_step), Path(args.out) if args.out else None,
                                Path(args.data) if args.data else None, args.force)
    except GoldenError as exc:
        print(f'golden_rebuild: REFUSED: {exc}', file=sys.stderr)
        return 1
    print(json.dumps({'status': summary['status'], 'repo': summary['repo'], 'runtime_s': summary['runtime_s'],
                      'wall_s': round(time.monotonic() - t0, 1), 'cycle_time_s': summary['program']['cycle_time_s'],
                      'seams': [(s['label'], s['coverage_pct'], s['max_travel_speed_mm_s']) for s in summary['seams']]}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
