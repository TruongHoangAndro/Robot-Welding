"""CP-60 / CP-61 routes: cell model import (S10, Admin) and job roles / axis pick / cross-check (S2, Engineer).

Every OCC step runs as a run (``awm_geometry.cli`` in the conda interpreter, ADR-15); this module only stores the
small JSON documents (roles.json, the roles template) and builds the command lines. Files (all mm):

* ``cell/cad/``: ``cell.step`` + ``cell_meta.json`` (up axis), ``cell_components.json``, ``cell_view.glb``,
  ``roles.json``, ``axis_pick.json``, ``cad_map.yaml`` (compiled), ``views/<key>/`` (component edge views),
  ``head.step`` + ``cad_map_head.yaml`` (Tool / TCP tab; placed by the Admin).
* ``cell/roles_template.json``: instance name -> role, applied on later imports (always reviewed).
* ``jobs/<job>/``: ``roles.json``, ``axis_pick.json``, ``cad_check.json``, ``cache/components.json``.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Any, Callable, Collection, Dict, List, Literal, Optional

import yaml
from fastapi import APIRouter, Depends, File, Form, Response, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from awm_core import cad_roles, jsonio
from awm_core.hashing import sha256_file

from awm_api import adapters, jobs
from awm_api.auth import User
from awm_api.problems import ProblemError
from awm_api.runner import RunBusyError, RunManager, StepsError
from awm_api.settings import Settings

CELL_WRITE = ('Admin',)
CELL_READ = ('Operator', 'Engineer', 'Admin')
JOB_EDIT = ('Engineer',)
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')
CELL_META = 'cell_meta.json'
CELL_VIEW = 'cell_view.glb'
HEAD_STEP = 'head.step'
HEAD_MAP = 'cad_map_head.yaml'
CELL_FILES = (cad_roles.CELL_TREE, CELL_VIEW, cad_roles.ROLES_FILE, cad_roles.AXIS_FILE, CELL_META)
VIEW_KEY = re.compile(r'^[0-9a-f]{12}$')
CREATED_BY = 'awm_api (CP-60)'

EdgeId = Field(pattern=r'^E[0-9]+$')


class Assignment(BaseModel):
    model_config = ConfigDict(extra='forbid')
    path: str = Field(min_length=1)
    role: Literal['robot', 'tool', 'rotary_fixture', 'part', 'static', 'exclude']


class CellRolesPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    assignments: List[Assignment]
    collision: Optional[Dict[str, Literal['hull', 'mesh']]] = None
    contact_tolerance_mm: Optional[float] = Field(default=None, ge=0)


class JobRolesPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    assignments: List[Assignment]


class ComponentView(BaseModel):
    model_config = ConfigDict(extra='forbid')
    component_path: str = Field(min_length=1)


class CellAxisPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    component_path: str = Field(min_length=1)
    edge_id: str = EdgeId
    reverse: bool = False


class JobAxisPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    edge_id: str = EdgeId
    reverse: bool = False


class HeadView(BaseModel):
    model_config = ConfigDict(extra='forbid')
    component: str = Field(min_length=1)


class TcpPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    component: str = Field(min_length=1)
    edge_id: str = EdgeId
    reverse: bool = False
    stickout_mm: float = Field(default=0.0, ge=0.0, le=50.0, allow_inf_nan=False)


class OverridePost(BaseModel):
    """CP-62 override request (Admin): the reason is recorded verbatim (e.g. the OI-23 CAD issue)."""

    model_config = ConfigDict(extra='forbid')
    reason: str = Field(min_length=1, max_length=2000, pattern=r'\S')


def _instance(path: str) -> str:
    """Instance name of a component path (last segment; names are never split on '/')."""
    return path.split(' / ')[-1]


def _key(text: str) -> str:
    return hashlib.sha256(text.encode('utf-8')).hexdigest()[:12]


def _write_json(path: Path, doc: Dict[str, Any], schema: Optional[str] = None) -> None:
    if schema:
        jsonio.write_json(path, doc, schema)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f'.{path.name}.tmp')
    tmp.write_text(json.dumps(doc, indent=1) + '\n')
    tmp.replace(path)


def head_tcp_record(path: Path) -> Optional[Dict[str, Any]]:
    """Taught / mapped TCP of the head record (CP-61) for S10 display: ``tool0_T_tcp_mm``, beam axis and the teach
    record (mm, quaternion xyzw); None when head.yaml is missing or unreadable."""
    try:
        doc = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(doc, dict) or not isinstance(doc.get('tool0_T_tcp_mm'), dict):
        return None
    ests = doc.get('estimates')
    est = ests.get('tcp') if isinstance(ests, dict) else None
    if not isinstance(est, dict):  # review minor 6: unexpected shapes give the mapped TCP only, never a 500
        est = {}
    out: Dict[str, Any] = {'method': est.get('method'), 'xyz_mm': doc['tool0_T_tcp_mm'].get('xyz'),
                           'quat_xyzw': doc['tool0_T_tcp_mm'].get('quat_xyzw'),
                           'beam_axis_tool0': doc.get('beam_axis_tool0')}
    for k in ('component', 'edge_id', 'reversed', 'stickout_mm', 'mouth_radius_mm', 'taught_by', 'taught_at'):
        if k in est:
            out[k] = est[k]
    return out


def roles_problem(problems: List[str]) -> ProblemError:
    return ProblemError(422, 'roles-invalid', 'Roles do not fit the file', detail='; '.join(problems),
                        errors=[{'pointer': '/assignments', 'message': p} for p in problems])


def is_assembly(job: Path) -> bool:
    """Job imported as an assembly (positioner + fixture + part, CP-60): it has a component tree."""
    return job.joinpath(*cad_roles.JOB_TREE).is_file()


OVERRIDE_FILE = 'cad_check_override.json'
OVERRIDE_SCHEMA = 'v2/cad_check_override'
OVERRIDE_CHECK = 4  # CP-62: only cross-check 4 may be overridden
OVERRIDE_BY = 'awm_api (CP-62 override of cross-check 4)'
CP62_HINT = ('an Admin may record a CP-62 override of check 4 for kinematic-only planning (collision off) until the '
             'cell CAD is fixed (OI-23)')


def red_checks(doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [c for c in doc.get('checks', []) if c.get('status') == 'red']


def check4_findings(doc: Dict[str, Any]) -> List[str]:
    """Names of the static objects hit by the swept rotating set (check 4 ``values.hits[].static``), else its note."""
    c4 = next((c for c in doc.get('checks', []) if c.get('id') == OVERRIDE_CHECK), {})
    names = sorted({str(h.get('static')) for h in (c4.get('values') or {}).get('hits', []) or []
                    if isinstance(h, dict) and h.get('static')})
    return names or [c4.get('note') or f'check {OVERRIDE_CHECK} red']


def _red_detail(reds: List[Dict[str, Any]]) -> str:
    return '; '.join(f"check {c['id']}: {c.get('note', c['status'])}" for c in reds)


def override_state(job: Path, admins: Collection[str]) -> Dict[str, Any]:
    """CP-62 override record of a job and whether it applies to the current ``cad_check.json``:
    ``{override, valid, problem, kind}``; kind = missing | invalid | void | ok. Invalid also when ``inputs[0]`` does
    not record the same hash as ``cad_check_sha256`` or ``by`` is not an Admin of the users store (``admins``)."""
    path = job / OVERRIDE_FILE
    doc = jobs.read_json(path)
    if doc is None:
        return {'override': None, 'valid': False, 'problem': None, 'kind': 'missing'}
    try:
        jsonio.validate(doc, OVERRIDE_SCHEMA)
    except jsonio.SchemaValidationError as exc:
        return {'override': doc, 'valid': False, 'kind': 'invalid',
                'problem': f'{OVERRIDE_FILE} does not match {OVERRIDE_SCHEMA}: {exc}'}
    inputs = doc.get('inputs') or [{}]
    if inputs[0].get('name') != cad_roles.CHECK_FILE or inputs[0].get('sha256') != doc['cad_check_sha256']:
        return {'override': doc, 'valid': False, 'kind': 'invalid',
                'problem': f'{OVERRIDE_FILE}: inputs[0] does not record {cad_roles.CHECK_FILE} with the hash in '
                           'cad_check_sha256'}
    if doc['by'] not in admins:
        return {'override': doc, 'valid': False, 'kind': 'invalid',
                'problem': f'{OVERRIDE_FILE}: recorded by {doc["by"]!r}, who is not an Admin of the users store'}
    check = job / cad_roles.CHECK_FILE
    if not check.is_file() or sha256_file(check) != doc['cad_check_sha256']:
        return {'override': doc, 'valid': False, 'kind': 'void',
                'problem': f'the cross-check changed after the override was recorded by {doc["by"]} at '
                           f'{doc["created_at"]}: the override is void; an Admin must review the new check and '
                           'record it again'}
    return {'override': doc, 'valid': True, 'problem': None, 'kind': 'ok'}


def override_reference(job: Path, state: Dict[str, Any]) -> Dict[str, Any]:
    """The D07 / D09 ``cad_check_override`` field (CP-62) for a valid override; who / when / reason are copied from
    the record so the audit trail survives a later DELETE or a new override (review I1)."""
    rec = state['override']
    return {'file': OVERRIDE_FILE, 'sha256': sha256_file(job / OVERRIDE_FILE),
            'cad_check_sha256': rec['cad_check_sha256'], 'check_id': OVERRIDE_CHECK,
            'by': rec['by'], 'created_at': rec['created_at'], 'reason': rec['reason']}


def _current_check(job: Path, repo_root: Path) -> Dict[str, Any]:
    doc = jobs.read_json(job / cad_roles.CHECK_FILE)
    if doc is None:
        raise ProblemError(409, 'cad-check-missing', 'Cross-check missing',
                           detail='run the CAD cross-check (S2) after roles and the positioner axis')
    if jobs.document_state(job, cad_roles.CHECK_FILE, repo_root, doc) == 'stale':
        raise ProblemError(409, 'cad-check-stale', 'Cross-check out of date',
                           detail='roles, axis pick, job STEP or cell model changed: run the cross-check again')
    return doc


def cad_gate(job: Path, repo_root: Path, admins: Collection[str], collision: str = 'off') -> Optional[Dict[str, Any]]:
    """CP-60 gate for seam picking and Plan: a job with roles needs a current cross-check that is not red.

    CP-62: a red check passes only when check 4 is the only red check, the check is current, a valid Admin override
    records the SHA-256 of this very ``cad_check.json`` and the request is kinematic only (collision ``off``). Returns
    the override reference for D07 / D09 in that case, else None."""
    if not (job / cad_roles.ROLES_FILE).is_file():
        if is_assembly(job):
            raise ProblemError(409, 'roles-missing', 'Roles missing',
                               detail='this job was imported as an assembly: save its roles, pick the positioner '
                                      'axis and run the CAD cross-check first (CP-60)')
        return None
    doc = _current_check(job, repo_root)
    if doc.get('status') != 'red':
        return None
    reds = red_checks(doc)
    only4 = bool(reds) and all(c.get('id') == OVERRIDE_CHECK for c in reds)
    state = override_state(job, admins) if only4 else {'kind': 'missing'}
    if state['kind'] == 'missing':
        detail = _red_detail(reds) or 'status red'
        if only4:
            detail = (f'check 4: static object(s) in the volume swept by the rotating set: '
                      f'{", ".join(check4_findings(doc))}. The cause is in the CAD (OI-23): separate them in the cell '
                      f'CAD or correct their roles (S10 cell roles / job roles); {CP62_HINT}')
        raise ProblemError(409, 'cad-check-red', 'Cross-check red', detail=detail)
    if state['kind'] == 'invalid':
        raise ProblemError(409, 'cad-check-override-invalid', 'Override record invalid', detail=state['problem'])
    if state['kind'] == 'void':
        raise ProblemError(409, 'cad-check-override-void', 'Override void (cross-check changed)',
                           detail=state['problem'])
    if collision != 'off':
        raise ProblemError(409, 'cad-check-override-kinematic-only', 'Override is for kinematic planning only',
                           detail='check 4 is red and overridden (CP-62) for kinematic-only planning: plan with '
                                  'collision off; collision-on planning and every release refuse while check 4 is red')
    return override_reference(job, state)


def register(api: APIRouter, settings: Settings, runs: RunManager, require: Callable[..., Any],
             busy: Callable[[RunBusyError], ProblemError], admins: Collection[str]) -> None:
    root = settings.repo_root
    cad = root.joinpath(*cad_roles.CELL_CAD_DIR)
    template_path = root.joinpath(*cad_roles.TEMPLATE_FILE)

    def start(job_id: str, kind: str, steps, on_success=None, on_failure=None) -> Dict[str, Any]:
        try:
            return runs.start(job_id, kind, steps, on_success=on_success, on_failure=on_failure)
        except RunBusyError as exc:
            raise busy(exc) from exc
        except StepsError as exc:
            raise ProblemError(500, 'cli-not-configured', detail=str(exc)) from exc

    def accepted(response: Response, run: Dict[str, Any], **extra) -> Dict[str, Any]:
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id'], **extra}

    def meta() -> Dict[str, Any]:
        doc = jobs.read_json(cad / CELL_META)
        if doc is None or not (cad / cad_roles.CELL_STEP).is_file():
            raise ProblemError(409, 'no-cell-cad', 'No cell CAD', detail='import the cell STEP first (S10)')
        return doc

    def tree(path: Path) -> List[Dict[str, Any]]:
        doc = jobs.read_json(path)
        if doc is None:
            raise ProblemError(409, 'no-tree', 'No component tree', detail=f'{path.name} missing: import first')
        return doc['components']

    def template() -> Dict[str, Any]:
        return jobs.read_json(template_path) or {'version': 0, 'roles': {}}

    def save_template(t: List[Dict[str, Any]], assignments: List[Dict[str, str]]) -> None:
        _write_json(template_path, cad_roles.update_template(template(), t, assignments))

    def roles_doc(step: Path, base: Path, up_axis: str, scope: str, body: Dict[str, Any]) -> Dict[str, Any]:
        doc = jsonio.make_header(CREATED_BY, [step], base)
        doc.update({'schema_version': '2.0', 'scope': scope,
                    'source': {'file': step.name, 'sha256': sha256_file(step), 'up_axis': up_axis},
                    'template_version': template().get('version')})
        doc.update({k: v for k, v in body.items() if v is not None})
        return doc

    def roles_state(path: Path, step: Path) -> str:
        doc = jobs.read_json(path)
        if doc is None:
            return 'missing'
        return 'ok' if (doc.get('source') or {}).get('sha256') == sha256_file(step) else 'stale'

    def roles_view(path: Path, step: Path, t: List[Dict[str, Any]]) -> Dict[str, Any]:
        if roles_state(path, step) == 'ok':
            doc = jobs.read_json(path)
            return {'assignments': doc['assignments'], 'prefilled': False,
                    'collision': doc.get('collision'), 'contact_tolerance_mm': doc.get('contact_tolerance_mm')}
        return {'assignments': cad_roles.apply_template(t, template()), 'prefilled': True}

    # -- cell model (S10) ---------------------------------------------------------------------------------------
    @api.post('/cell/model', status_code=202)
    def import_cell(response: Response, step: UploadFile = File(...), up_axis: str = Form(...),
                    user: User = Depends(require(*CELL_WRITE))) -> Dict[str, str]:
        if up_axis not in UP_AXES:
            raise ProblemError(400, 'bad-request', detail=f'up_axis must be one of {", ".join(UP_AXES)}')
        if not (step.filename or '').lower().endswith(('.step', '.stp')):
            raise ProblemError(400, 'bad-request', detail='a STEP file (.step / .stp) is required')

        def build(run_dir: Path):
            upload = run_dir / 'upload' / cad_roles.CELL_STEP
            adapters.save_upload(step, upload, settings.max_upload_mb)
            return adapters.cell_import_steps(settings, upload, run_dir / 'out', up_axis)

        def promote(run_dir: Path) -> Dict[str, str]:
            cad.mkdir(parents=True, exist_ok=True)
            (run_dir / 'upload' / cad_roles.CELL_STEP).replace(cad / cad_roles.CELL_STEP)
            (run_dir / 'out' / cad_roles.CELL_TREE).replace(cad / cad_roles.CELL_TREE)
            (run_dir / 'out' / 'view' / CELL_VIEW).replace(cad / CELL_VIEW)
            _write_json(cad / CELL_META, {'file': step.filename, 'sha256': sha256_file(cad / cad_roles.CELL_STEP),
                                          'up_axis': up_axis, 'imported_by': user.username,
                                          'imported_at': jsonio.utc_now_iso()})
            return {'cell.step': sha256_file(cad / cad_roles.CELL_STEP)}

        return accepted(response, start('cell', 'cell-import', build, on_success=promote))

    @api.get('/cell/model')
    def cell_status(_u: User = Depends(require(*CELL_READ))) -> Dict[str, Any]:
        doc = jobs.read_json(cad / CELL_META)
        step = cad / cad_roles.CELL_STEP
        out: Dict[str, Any] = {'step': None, 'up_axis': None, 'roles': {'state': 'missing'}, 'axis': None,
                               'model': {'saved': (settings.cell_model_dir / 'frames.yaml').is_file()},
                               'head': {'step': (cad / HEAD_STEP).is_file(), 'map': (cad / HEAD_MAP).is_file(),
                                        'head_yaml': (settings.head_dir / 'head.yaml').is_file(),
                                        'tcp': head_tcp_record(settings.head_dir / 'head.yaml')},
                               'stale_jobs': []}
        if doc is not None and step.is_file():
            out.update(step={'file': doc.get('file'), 'sha256': sha256_file(step)}, up_axis=doc.get('up_axis'),
                       roles={'state': roles_state(cad / cad_roles.ROLES_FILE, step)})
            pick = jobs.read_json(cad / cad_roles.AXIS_FILE)
            if pick is not None:
                out['axis'] = {k: pick.get(k) for k in ('component_path', 'edge_id', 'center_file_mm', 'axis_file',
                                                         'radius_mm', 'reversed')}
                out['axis']['state'] = 'ok' if (pick.get('source') or {}).get('sha256') == sha256_file(step) \
                    else 'stale'
        if settings.jobs_dir.is_dir():
            out['stale_jobs'] = sorted(j.name for j in settings.jobs_dir.iterdir()
                                       if j.is_dir() and not j.name.startswith('.')
                                       and jobs.document_state(j, 'placement.json', root) == 'stale')
        return out

    @api.get('/cell/model/files/{name}')
    def cell_file(name: Literal[CELL_FILES], _u: User = Depends(require(*CELL_READ))):  # type: ignore[valid-type]
        from fastapi.responses import FileResponse
        path = cad / name
        if not path.is_file():
            raise ProblemError(404, 'not-found', detail=f'{name} not present')
        media = 'model/gltf-binary' if name.endswith('.glb') else 'application/json'
        return FileResponse(path, media_type=media, headers={'Cache-Control': 'no-store'})

    @api.get('/cell/model/roles')
    def cell_roles_get(_u: User = Depends(require(*CELL_READ))) -> Dict[str, Any]:
        meta()
        return roles_view(cad / cad_roles.ROLES_FILE, cad / cad_roles.CELL_STEP, tree(cad / cad_roles.CELL_TREE))

    @api.put('/cell/model/roles')
    def cell_roles_put(body: CellRolesPut, _u: User = Depends(require(*CELL_WRITE))) -> Dict[str, Any]:
        m = meta()
        t = tree(cad / cad_roles.CELL_TREE)
        data = body.model_dump()
        problems = cad_roles.validate_roles(t, data, 'cell')
        if problems:
            raise roles_problem(problems)
        doc = roles_doc(cad / cad_roles.CELL_STEP, cad, m['up_axis'], 'cell', data)
        _write_json(cad / cad_roles.ROLES_FILE, doc, 'v2/cad_roles')
        save_template(t, data['assignments'])
        return doc

    def cached_view(key: str, step: Path, component: str, up_axis: str) -> Optional[JSONResponse]:
        """Close-out m5: reuse ``views/<key>/`` when it was exported from this STEP (hash), component and up axis."""
        topo = jobs.read_json(cad / 'views' / key / 'topology.json')
        if topo is None or not (cad / 'views' / key / 'view.glb').is_file() or not step.is_file():
            return None
        src = topo.get('source') or {}
        digest = sha256_file(step)
        if (src.get('component') == component and src.get('up_axis') == up_axis
                and any(i.get('sha256') == digest for i in topo.get('inputs') or [])):
            return JSONResponse({'key': key, 'cached': True}, status_code=200)
        return None

    @api.post('/cell/model/component_view', status_code=202)
    def cell_component_view(body: ComponentView, response: Response,
                            _u: User = Depends(require(*CELL_WRITE))):
        m = meta()
        key = _key(body.component_path)
        hit = cached_view(key, cad / cad_roles.CELL_STEP, _instance(body.component_path), m['up_axis'])
        if hit is not None:
            return hit
        steps = adapters.component_view_steps(settings, cad / cad_roles.CELL_STEP, _instance(body.component_path),
                                              m['up_axis'], cad / 'views' / key)
        return accepted(response, start('cell', 'cell-view', steps), key=key)

    @api.get('/cell/model/views/{key}/{file}')
    def cell_view_file(key: str, file: Literal['view.glb', 'topology.json'],
                       _u: User = Depends(require(*CELL_READ))):
        from fastapi.responses import FileResponse
        path = cad / 'views' / key / file
        if not VIEW_KEY.match(key) or not path.is_file():
            raise ProblemError(404, 'not-found', detail=f'view {key}/{file} not exported')
        media = 'model/gltf-binary' if file.endswith('.glb') else 'application/json'
        return FileResponse(path, media_type=media, headers={'Cache-Control': 'no-store'})

    @api.put('/cell/model/axis', status_code=202)
    def cell_axis_put(body: CellAxisPut, response: Response,
                      _u: User = Depends(require(*CELL_WRITE))) -> Dict[str, str]:
        m = meta()
        steps = adapters.pick_circle_steps(settings, cad / cad_roles.CELL_STEP, body.edge_id, m['up_axis'],
                                           cad / cad_roles.AXIS_FILE, component=_instance(body.component_path),
                                           reverse=body.reverse)
        return accepted(response, start('cell', 'cell-axis', steps))

    @api.post('/cell/model/save', status_code=202)
    def cell_save(response: Response, _u: User = Depends(require(*CELL_WRITE))) -> Dict[str, str]:
        meta()
        missing = [n for n in (cad_roles.ROLES_FILE, cad_roles.AXIS_FILE) if not (cad / n).is_file()]
        if missing:
            raise ProblemError(409, 'cell-incomplete', 'Cell model incomplete',
                               detail='set the roles and pick the positioner axis first (missing: '
                                      + ', '.join(missing) + ')')
        steps = adapters.cell_save_steps(settings, cad, root)
        return accepted(response, start('cell', 'cell-save', steps))

    # -- Tool / TCP tab (CP-61) ---------------------------------------------------------------------------------
    def head_files() -> None:
        if not (cad / HEAD_STEP).is_file() or not (cad / HEAD_MAP).is_file():
            raise ProblemError(409, 'no-head-cad', 'No head CAD',
                               detail=f'put the head STEP and its map into cell/cad ({HEAD_STEP}, {HEAD_MAP})')

    @api.post('/cell/head/view', status_code=202)
    def head_view(body: HeadView, response: Response, _u: User = Depends(require(*CELL_WRITE))):
        head_files()
        key = _key(f'head:{body.component}')
        hit = cached_view(key, cad / HEAD_STEP, body.component, '+Z')
        if hit is not None:
            return hit
        steps = adapters.component_view_steps(settings, cad / HEAD_STEP, body.component, '+Z', cad / 'views' / key)
        return accepted(response, start('cell', 'head-view', steps), key=key)

    @api.put('/cell/head/tcp', status_code=202)
    def head_tcp(body: TcpPut, response: Response, user: User = Depends(require(*CELL_WRITE))) -> Dict[str, str]:
        head_files()
        steps = adapters.tcp_teach_steps(settings, cad / HEAD_STEP, cad / HEAD_MAP, root, body.component,
                                         body.edge_id, body.reverse, body.stickout_mm, user.username)
        return accepted(response, start('cell', 'tcp-teach', steps))

    # -- job roles / axis / cross-check (S2) --------------------------------------------------------------------
    def job_up_axis(job: Path) -> str:
        topo = jobs.read_json(job / 'cache' / 'view' / 'topology.json') or {}
        return (topo.get('source') or {}).get('up_axis') or '+Z'

    @api.get('/jobs/{job_id}/roles')
    def job_roles_get(job_id: str, _u: User = Depends(require(*CELL_READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        return roles_view(job / cad_roles.ROLES_FILE, job / 'model.step', tree(job.joinpath(*cad_roles.JOB_TREE)))

    @api.put('/jobs/{job_id}/roles')
    def job_roles_put(job_id: str, body: JobRolesPut, response: Response,
                      _u: User = Depends(require(*JOB_EDIT))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        t = tree(job.joinpath(*cad_roles.JOB_TREE))
        data = body.model_dump()
        problems = cad_roles.validate_roles(t, data, 'job')
        if problems:
            raise roles_problem(problems)
        doc = roles_doc(job / 'model.step', job, job_up_axis(job), 'job', data)
        try:
            with runs.slot('roles', job_id):
                _write_json(job / cad_roles.ROLES_FILE, doc, 'v2/cad_roles')
        except RunBusyError as exc:
            raise busy(exc) from exc
        save_template(t, data['assignments'])
        response.headers['X-AWM-Seam-Axis'] = jobs.seam_axis_state(job, root)
        return doc

    @api.put('/jobs/{job_id}/axis', status_code=202)
    def job_axis_put(job_id: str, body: JobAxisPut, response: Response,
                     _u: User = Depends(require(*JOB_EDIT))) -> Dict[str, str]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        steps = adapters.pick_circle_steps(settings, job / 'model.step', body.edge_id, job_up_axis(job),
                                           job / cad_roles.AXIS_FILE, components=True, reverse=body.reverse)
        return accepted(response, start(job_id, 'axis', steps))

    @api.post('/jobs/{job_id}/check', status_code=202)
    def job_check(job_id: str, response: Response, _u: User = Depends(require(*JOB_EDIT))) -> Dict[str, str]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        missing = [n for n in (cad_roles.ROLES_FILE, cad_roles.AXIS_FILE) if not (job / n).is_file()]
        if missing:
            raise ProblemError(409, 'job-cad-incomplete', 'Roles or axis missing',
                               detail='set the roles and pick the positioner axis first (missing: '
                                      + ', '.join(missing) + ')')
        return accepted(response, start(job_id, 'cad-check', adapters.job_placement_steps(settings, job_id)))

    @api.get('/jobs/{job_id}/check')
    def job_check_get(job_id: str, _u: User = Depends(require(*CELL_READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        doc = jobs.read_json(job / cad_roles.CHECK_FILE)
        if doc is None:
            raise ProblemError(404, 'no-check', 'No cross-check', detail='run the CAD cross-check first')
        return {'state': jobs.document_state(job, cad_roles.CHECK_FILE, root, doc), 'check': doc}

    # -- CP-62: recorded Admin override of cross-check 4 (kinematic-only planning) -----------------------------
    @api.get('/jobs/{job_id}/check/override')
    def override_get(job_id: str, _u: User = Depends(require(*CELL_READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        state = override_state(job, admins)
        if state['kind'] == 'missing':
            raise ProblemError(404, 'no-override', 'No override', detail='no CP-62 override recorded for this job')
        return {k: state[k] for k in ('override', 'valid', 'problem')}

    @api.post('/jobs/{job_id}/check/override', status_code=201)
    def override_post(job_id: str, body: OverridePost, user: User = Depends(require(*CELL_WRITE))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        try:
            with runs.slot('override', job_id):
                doc = _current_check(job, root)
                reds = red_checks(doc)
                others = [c for c in reds if c.get('id') != OVERRIDE_CHECK]
                if doc.get('status') != 'red' or not reds or others:
                    raise ProblemError(409, 'override-not-applicable', 'Override not applicable',
                                       detail='CP-62 overrides check 4 only, and only when it is the only red check'
                                              + (f'; still red: {_red_detail(others)}' if others else
                                                 '; check 4 is not red'))
                check = job / cad_roles.CHECK_FILE
                rec = jsonio.make_header(OVERRIDE_BY, [check], job)
                rec.update({'schema_version': '2.0', 'by': user.username, 'role': 'admin', 'reason': body.reason.strip(),
                            'check_id': OVERRIDE_CHECK, 'cad_check_sha256': sha256_file(check),
                            'scope': 'kinematic_planning', 'findings': check4_findings(doc)})
                _write_json(job / OVERRIDE_FILE, rec, OVERRIDE_SCHEMA)
        except RunBusyError as exc:
            raise busy(exc) from exc
        return rec

    @api.delete('/jobs/{job_id}/check/override', status_code=204)
    def override_delete(job_id: str, _u: User = Depends(require(*CELL_WRITE))) -> Response:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        try:
            with runs.slot('override', job_id):
                if not (job / OVERRIDE_FILE).is_file():
                    raise ProblemError(404, 'no-override', 'No override', detail='no CP-62 override recorded')
                (job / OVERRIDE_FILE).unlink()
        except RunBusyError as exc:
            raise busy(exc) from exc
        return Response(status_code=204)
