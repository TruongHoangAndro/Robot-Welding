"""FastAPI application: ``/api/v1`` (contract docs/generated/contracts/awm_api_sim1.openapi.yaml) + static front-end.

Roles (DES-001 §7.3, §4.5 S1 / S2 / S4, CP-21): every role reads; jobs are created by Engineer or Admin (S1); seams
and the placement are edited by Engineer only (CP-21, §7.3 "placement study"); planning and program runs are started
by Engineer (§7.3 "run simulation"); runs are cancelled by Engineer or Admin. Nothing here commands the PLC or the
laser: S4 playback is display only, computed in the browser from the stored program. The one motion request is the S4
"Replay in RViz" (G1.20, Engineer, §7.3 "run simulation and dry run" in simulation): a run of the replay CLI against
the sim_mock the user launched; the CLI refuses anything but mock hardware and this API cannot bypass it.
"""

from __future__ import annotations

import asyncio
import contextlib
import datetime as _dt
import json
import logging
import re
import secrets
import shutil
from pathlib import Path
from typing import Annotated, Any, Dict, List, Literal, Optional

import numpy as np
from fastapi import APIRouter, Depends, FastAPI, File, Form, Header, Request, Response, UploadFile, WebSocket
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator
from starlette.websockets import WebSocketDisconnect

from awm_api import adapters, cad_api, jobs, jog, placement as placement_mod, replay as replay_mod
from awm_api.auth import TokenStore, User, forbidden, load_users, unauthorized, verify_password
from awm_api.problems import ProblemError, install as install_problems
from awm_api.robot_model import API_MESHES, RobotModel, safe_child
from awm_api.runner import TERMINAL, RunBusyError, RunFinishedError, RunManager, RunNotFoundError, StepsError
from awm_api.settings import Settings
from awm_path.core.kinematics import UrAnalyticKinematics

LOG = logging.getLogger('awm_api')

# Role ranks: roles are hierarchical, Admin ⊇ Engineer ⊇ Operator (CP-63, Truong 2026-11-09). A route lists the
# lowest role it needs; a higher role is always allowed (see ``require``). An Admin-only route still lists only
# ('Admin',), so an Engineer cannot reach it.
RANK = {'Operator': 0, 'Engineer': 1, 'Admin': 2}

READ = ('Operator', 'Engineer', 'Admin')
CREATE_JOB = ('Engineer', 'Admin')
EDIT_SEAMS = ('Engineer',)
EDIT_PLACEMENT = ('Engineer',)
PLAN = ('Engineer',)
CANCEL = ('Engineer', 'Admin')
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')
STEP_SUFFIXES = ('.step', '.stp')

SeamId = Annotated[str, Field(pattern=r'^S[0-9]{2,3}$')]
EdgeId = Annotated[str, Field(pattern=r'^E[0-9]+$')]


class JogPose(BaseModel):
    """Pose in mm / unit quaternion (CP-64)."""
    model_config = ConfigDict(extra='forbid')
    xyz_mm: List[float] = Field(min_length=3, max_length=3)
    quat_xyzw: List[float] = Field(min_length=4, max_length=4)


class JogFk(BaseModel):
    model_config = ConfigDict(extra='forbid')
    joints_deg: List[float] = Field(min_length=6, max_length=6)


class HomeBody(BaseModel):
    """PUT /cell/home: joint set and positioner angle are checked by the ``home`` schema (422 on mismatch)."""
    model_config = ConfigDict(extra='forbid')
    robot_joints_deg: Dict[str, Any]
    positioner_joint_deg: Any


class JogIk(BaseModel):
    model_config = ConfigDict(extra='forbid')
    target_tcp_laser: JogPose
    current_joints_deg: List[float] = Field(min_length=6, max_length=6)


class Login(BaseModel):
    model_config = ConfigDict(extra='forbid')
    username: str = Field(min_length=1)
    password: str = Field(min_length=1)


class SeamPick(BaseModel):
    """Contract SeamPick + the D04 2.1 fields of G1.2 (CP-59): positioner mode / angles, start offset."""

    model_config = ConfigDict(extra='forbid')
    seam_id: SeamId
    edges: List[EdgeId] = Field(min_length=1)
    reversed: bool
    range_mm: Optional[List[float]] = Field(default=None, min_length=2, max_length=2)
    joint_type: Literal['butt', 'fillet', 'lap', 'groove']
    sequence: int = Field(ge=1)
    leg_mm: Optional[float] = Field(default=None, ge=0)
    arcs: int = Field(default=1, ge=1, le=12)
    # Assumption: 2 mm arc overlap when the client sends none (the WPS default is not available in Goal 1).
    arc_overlap_mm: float = Field(default=2.0, ge=0)
    positioner_mode: Literal['sync', 'C', 'B'] = 'sync'
    positioner_angles_deg: Optional[List[float]] = None
    start_offset_mm: float = Field(default=0.0, ge=0)
    note: Optional[str] = None


class SeamsPut(BaseModel):
    model_config = ConfigDict(extra='forbid')
    seams: List[SeamPick]


class PlanRequest(BaseModel):
    """Plan request. ``travel_speed_mm_s``: WPS travel speed relative to the part, entered by the Engineer (CP-59
    follow-up 2026-11-06); absent = the planner default (Assumption, recorded in D07). Never slowed down silently:
    the planner reports the largest feasible speed per arc and the program step refuses a speed above it."""

    model_config = ConfigDict(extra='forbid')
    seams: Optional[List[SeamId]] = None
    travel_speed_mm_s: Optional[float] = Field(default=None, gt=0, allow_inf_nan=False)
    #: Goal 1 plans kinematic only (CP-58); ``on`` is refused (409) until collision planning exists (Goal 2), and is
    #: refused separately while a CP-62 override of check 4 is what lets the job plan.
    collision: Literal['off', 'on'] = 'off'


Coord = Annotated[float, Field(ge=-placement_mod.MAX_OFFSET_MM, le=placement_mod.MAX_OFFSET_MM,
                               allow_inf_nan=False)]
Finite = Annotated[float, Field(allow_inf_nan=False)]


class PoseIn(BaseModel):
    """Pose of the part frame in ``positioner_flange``: mm and either roll-pitch-yaw in degrees (URDF convention,
    R = Rz(yaw)·Ry(pitch)·Rx(roll)) or a quaternion xyzw."""

    model_config = ConfigDict(extra='forbid')
    position_mm: List[Coord] = Field(min_length=3, max_length=3)
    rpy_deg: Optional[List[Finite]] = Field(default=None, min_length=3, max_length=3)
    quat_xyzw: Optional[List[Finite]] = Field(default=None, min_length=4, max_length=4)

    @model_validator(mode='after')
    def _one_rotation(self) -> 'PoseIn':
        if (self.rpy_deg is None) == (self.quat_xyzw is None):
            raise ValueError('give exactly one of rpy_deg or quat_xyzw')
        return self


class PlacementPut(BaseModel):
    """Manual placement (``flange_T_part``) or a copy of another job's ``flange_T_part`` (``copy_from``)."""

    model_config = ConfigDict(extra='forbid')
    flange_T_part: Optional[PoseIn] = None
    copy_from: Optional[str] = Field(default=None, pattern=jobs.JOB_ID_RE.pattern)

    @model_validator(mode='after')
    def _one_source(self) -> 'PlacementPut':
        if (self.flange_T_part is None) == (self.copy_from is None):
            raise ValueError('give exactly one of flange_T_part or copy_from')
        return self


class ProgramRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')


class ReplayRequest(BaseModel):
    """S4 replay: the newest program revision unless one is named; it is checked for staleness either way."""

    model_config = ConfigDict(extra='forbid')
    program_rev: Optional[str] = Field(default=None, pattern=r'^P[0-9]{3}$')


def _check_picks(picks: List[SeamPick]) -> None:
    errors = []
    seen = set()
    for i, p in enumerate(picks):
        if p.seam_id in seen:
            errors.append({'pointer': f'/seams/{i}/seam_id', 'message': f'duplicate seam id {p.seam_id}'})
        seen.add(p.seam_id)
        if p.range_mm is not None:
            errors.append({'pointer': f'/seams/{i}/range_mm', 'message': 'range_mm is not supported in Goal 1 '
                           '(D04 2.1): use arcs, arc_overlap_mm and start_offset_mm'})
        ptr = f'/seams/{i}/positioner_angles_deg'
        if p.positioner_mode == 'B':
            if p.positioner_angles_deg is None or len(p.positioner_angles_deg) != p.arcs:
                errors.append({'pointer': ptr, 'message': f'mode B needs one positioner angle per arc ({p.arcs})'})
        elif p.positioner_angles_deg is not None:
            errors.append({'pointer': ptr, 'message': 'positioner angles are entered only in mode B'})
    if errors:
        raise ProblemError(422, 'validation', 'Seam list invalid',
                           detail='; '.join(f"{e['pointer']}: {e['message']}" for e in errors), errors=errors)


def _busy(exc: RunBusyError) -> ProblemError:
    return ProblemError(409, 'run-busy', 'Cell busy', detail=f'one run per cell: {exc.active_run} is active',
                        active_run=exc.active_run)


def create_app(settings: Settings) -> FastAPI:
    users = load_users(settings.secrets_file)  # fails early with a clear message
    admins = frozenset(name for name, u in users.items() if u.role == 'Admin')  # CP-62 override authors
    tokens = TokenStore(settings.token_ttl_s)
    runs = RunManager(settings.runs_dir, kill_grace_s=settings.kill_grace_s)
    robot = RobotModel(settings)
    kin = UrAnalyticKinematics()  # CP-64 jog page: in-process analytic FK / IK (mm / deg)
    settings.jobs_dir.mkdir(parents=True, exist_ok=True)

    @contextlib.asynccontextmanager
    async def lifespan(_app: FastAPI):
        yield
        runs.shutdown()

    app = FastAPI(title='APLSMN601 awm_api', version='1.0.0-draft', lifespan=lifespan,
                  docs_url='/api/v1/docs', openapi_url='/api/v1/openapi.json', redoc_url=None)
    install_problems(app)
    app.state.settings, app.state.runs = settings, runs

    def current_user(authorization: Optional[str] = Header(default=None)) -> User:
        if not authorization or not authorization.lower().startswith('bearer '):
            raise unauthorized('bearer token missing; log in at /api/v1/auth/login')
        user = tokens.user(authorization.split(None, 1)[1].strip())
        if user is None:
            raise unauthorized('token unknown or expired; log in again')
        return user

    def require(*roles: str):
        # Allow any role at or above the lowest listed role (CP-63: Admin ⊇ Engineer ⊇ Operator). An Admin-only route
        # lists only ('Admin',), so the threshold is Admin and lower roles stay out.
        threshold = min(RANK[r] for r in roles)

        def dep(user: User = Depends(current_user)) -> User:
            if RANK[user.role] < threshold:
                raise forbidden(user, roles)
            return user
        return dep

    api = APIRouter(prefix='/api/v1')

    # -- auth -----------------------------------------------------------------------------------------------------
    @api.post('/auth/login')
    def login(body: Login) -> Dict[str, Any]:
        user = users.get(body.username)
        if user is None or not verify_password(body.password, user.password):
            raise ProblemError(401, 'login-failed', 'Login failed', detail='unknown user or wrong password')
        token, expires = tokens.issue(user)
        LOG.info('login %s (%s)', user.username, user.role)
        return {'token': token, 'role': user.role, 'expires_at': expires, 'username': user.username}

    # -- jobs -----------------------------------------------------------------------------------------------------
    @api.get('/jobs')
    def list_jobs(_u: User = Depends(require(*READ))) -> List[Dict[str, Any]]:
        out = {p.name: jobs.summary(p) for p in sorted(settings.jobs_dir.iterdir())
               if p.is_dir() and not p.name.startswith('.') and (p / 'part.json').is_file()}
        active = runs.active()
        if active and active['kind'] == 'import' and active['job_id'] not in out:
            no, _, rev = active['job_id'].rpartition('_')
            out[active['job_id']] = {'job_id': active['job_id'], 'part_no': no, 'part_rev': rev,
                                     'status': 'importing', 'updated_at': runs.get(active['run_id'])['started_at']}
        return [out[k] for k in sorted(out)]

    @api.post('/jobs', status_code=202)
    def create_job(response: Response, step: UploadFile = File(...), part_no: str = Form(...),
                   part_rev: str = Form(...), component: Optional[str] = Form(None), up_axis: str = Form('+Z'),
                   assembly: bool = Form(False),
                   _u: User = Depends(require(*CREATE_JOB))) -> Dict[str, str]:
        errors = [{'pointer': f'/{k}', 'message': f'{v!r} must match {jobs.TOKEN_RE.pattern}'}
                  for k, v in (('part_no', part_no), ('part_rev', part_rev)) if not jobs.TOKEN_RE.match(v)]
        if up_axis not in UP_AXES:
            errors.append({'pointer': '/up_axis', 'message': f'one of {", ".join(UP_AXES)}'})
        name = Path(step.filename or 'model.step').name
        if not name.lower().endswith(STEP_SUFFIXES):
            errors.append({'pointer': '/step', 'message': 'a STEP file (.step / .stp) is required'})
        if errors:
            raise ProblemError(400, 'bad-request', detail='; '.join(f"{e['pointer']}: {e['message']}"
                                                                     for e in errors), errors=errors)
        job_id = f'{part_no}_{part_rev}'
        if (settings.jobs_dir / job_id).exists():
            raise ProblemError(409, 'job-exists', detail=f'job {job_id} exists; use a new part revision')
        safe_name = re.sub(r'[^A-Za-z0-9._ -]', '_', name)

        def build(run_dir: Path):
            target = run_dir / 'upload' / safe_name
            target.parent.mkdir()
            limit, size = int(settings.max_upload_mb * 1024 * 1024), 0
            with target.open('wb') as fh:
                while chunk := step.file.read(1024 * 1024):
                    size += len(chunk)
                    if size > limit:
                        raise ProblemError(413, 'upload-too-large',
                                           detail=f'STEP larger than {settings.max_upload_mb:.0f} MB')
                    fh.write(chunk)
            return adapters.import_steps(settings, run_dir, target, part_no, part_rev, component or None, up_axis,
                                         components=assembly)

        try:
            run = runs.start(job_id, 'import', build,
                             on_success=lambda d: adapters.promote_import(settings, d, job_id),
                             on_failure=lambda d: adapters.discard_import(settings, d))
        except RunBusyError as exc:
            raise _busy(exc) from exc
        except StepsError as exc:
            raise ProblemError(500, 'cli-not-configured', detail=str(exc)) from exc
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id']}

    @api.get('/jobs/{job_id}')
    def get_job(job_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        active = runs.active()
        return jobs.detail(job, settings.repo_root,
                           active['run_id'] if active and active['job_id'] == job_id else None)

    @api.delete('/jobs/{job_id}', status_code=204)
    def delete_job(job_id: str, _u: User = Depends(require(*CREATE_JOB))) -> Response:
        job = jobs.job_dir(settings.jobs_dir, job_id)  # 400 bad id / 404 not found
        active = runs.active()
        if active and active['job_id'] == job_id:  # never delete the folder under a running job
            raise _busy(RunBusyError(active['run_id']))
        shutil.rmtree(job)
        LOG.info('job %s deleted by %s', job_id, _u.username)
        return Response(status_code=204)

    @api.get('/jobs/{job_id}/view/{file}')
    def get_view(job_id: str, file: Literal['view.glb', 'topology.json'],
                 _u: User = Depends(require(*READ))) -> FileResponse:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        path = job / 'cache' / 'view' / file
        if not path.is_file():
            raise ProblemError(404, 'view-not-found', detail=f'{file} not exported for job {job_id} (D20)')
        media = 'model/gltf-binary' if file.endswith('.glb') else 'application/json'
        return FileResponse(path, media_type=media, headers={'Cache-Control': 'no-store'})

    @api.get('/jobs/{job_id}/placement')
    def get_placement(job_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        doc = jobs.read_json(job / 'placement.json')
        if doc is None:
            raise ProblemError(404, 'no-placement', 'No placement', detail=jobs.no_placement_note(job))
        return doc

    @api.put('/jobs/{job_id}/placement')
    def put_placement(job_id: str, body: PlacementPut, response: Response,
                      _u: User = Depends(require(*EDIT_PLACEMENT))) -> Dict[str, Any]:
        """Manual pose (Assumption, ``source: study``) or copy from another job; makes seam_axis / path stale."""
        job = jobs.job_dir(settings.jobs_dir, job_id)
        current = jobs.read_json(job / 'placement.json') or {}
        if current.get('source') == 'job_cad' or cad_api.is_assembly(job):
            raise ProblemError(409, 'placement-from-cad', 'Placement comes from the job CAD',
                               detail='this job is placed from its STEP (source job_cad, CP-60): change the roles '
                                      'or the positioner axis and run the cross-check instead')
        if not (job / placement_mod.VIEW_MESH).is_file():
            raise ProblemError(409, 'no-view', 'No view model',
                               detail=f'{placement_mod.VIEW_MESH} is missing; the import (D20) did not finish')
        extra: List[Path] = []
        if body.copy_from is not None:
            if body.copy_from == job_id:
                raise ProblemError(422, 'validation', detail='copy_from names this job itself',
                                   errors=[{'pointer': '/copy_from', 'message': 'another job is needed'}])
            src = jobs.job_dir(settings.jobs_dir, body.copy_from)
            src_doc = jobs.read_json(src / 'placement.json')
            if src_doc is None or 'flange_T_part' not in src_doc:
                raise ProblemError(409, 'no-placement', 'No placement',
                                   detail=f'job {body.copy_from} has no placement.json to copy')
            pose, how = src_doc['flange_T_part'], f'flange_T_part copied from job {body.copy_from}'
            extra.append(src / 'placement.json')
        else:
            p = body.flange_T_part
            try:
                quat = placement_mod.quat_from_input(p.rpy_deg, p.quat_xyzw)
            except ValueError as exc:
                raise ProblemError(422, 'validation', detail=str(exc),
                                   errors=[{'pointer': '/flange_T_part/quat_xyzw', 'message': str(exc)}]) from exc
            pose, how = {'position_mm': list(p.position_mm), 'quat_xyzw': quat}, 'manual pose'
        try:
            with runs.slot('placement', job_id):
                doc = placement_mod.build(job, pose, how, extra)
                placement_mod.write(job, doc)
        except RunBusyError as exc:
            raise _busy(exc) from exc
        except ValueError as exc:  # schema refusal of the built document
            raise ProblemError(422, 'validation', 'Placement refused', detail=str(exc)) from exc
        response.headers['X-AWM-Seam-Axis'] = jobs.seam_axis_state(job, settings.repo_root)
        return doc

    @api.get('/jobs/{job_id}/files/{path:path}')
    def get_file(job_id: str, path: str, _u: User = Depends(require(*READ))) -> FileResponse:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        return FileResponse(jobs.safe_file(job, path), headers={'Cache-Control': 'no-store'})

    @api.get('/jobs/{job_id}/seams')
    def get_seams(job_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        doc = jobs.read_json(job / 'seams.json')
        if doc is None:
            raise ProblemError(404, 'no-seams', 'No seams', detail=f'no seams saved for job {job_id} yet')
        return doc

    @api.put('/jobs/{job_id}/seams')
    def put_seams(job_id: str, body: SeamsPut, response: Response,
                  _u: User = Depends(require(*EDIT_SEAMS))) -> Dict[str, Any]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        _check_picks(body.seams)
        cad_api.cad_gate(job, settings.repo_root, admins)  # CP-60: no seam picking on a red / stale cross-check
        has_placement = (job / 'placement.json').is_file()
        stamp = _dt.datetime.now(_dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        work = settings.repo_root / 'logs' / 'seam_saves' / f'{job_id}-{stamp}-{secrets.token_hex(3)}'
        picks = [p.model_dump() for p in body.seams]
        try:
            with runs.slot('seams', job_id):
                cad_api.cad_gate(job, settings.repo_root, admins)  # again inside the slot (review minor 3)
                doc = adapters.build_seams(settings, job, picks, work, build_axis=has_placement)
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['X-AWM-Seam-Axis'] = (
            'built' if has_placement else 'not built: no placement.json (' + jobs.placement_hint(job) + ')')
        return doc

    def promote_gated(run_dir: Path, job: Path, collision: str, override: Optional[Dict[str, Any]]) -> Dict[str, str]:
        """Plan promotion with the CP-60 / CP-62 gate evaluated again while the run still holds the slot (review
        minor 3): a cross-check or override that became stale, red or void during the run is never promoted."""
        again = cad_api.cad_gate(job, settings.repo_root, admins, collision)
        if again != override:
            raise RuntimeError('the CP-62 override changed during the run; nothing promoted')
        return adapters.promote_plan(run_dir, job, override)

    @api.post('/jobs/{job_id}/plan', status_code=202)
    def plan(job_id: str, response: Response, body: Optional[PlanRequest] = None,
             _u: User = Depends(require(*PLAN))) -> Dict[str, str]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        collision = body.collision if body else 'off'
        # CP-60: Plan blocked by a red / stale cross-check; CP-62: a valid check-4 override lets a kinematic plan pass
        override = cad_api.cad_gate(job, settings.repo_root, admins, collision)
        if collision != 'off':
            raise ProblemError(409, 'collision-unavailable', 'Collision planning not available',
                               detail='Goal 1 plans kinematic only (CP-58, collision_checked false); collision-on '
                                      'planning comes with Goal 2')
        seams_doc = jobs.read_json(job / 'seams.json')
        if seams_doc is None:
            raise ProblemError(409, 'no-seams', 'No seams', detail='save the seams (S2) before planning')
        if not (job / 'placement.json').is_file():
            raise ProblemError(409, 'no-placement', 'No placement', detail=jobs.no_placement_note(job))
        axis = jobs.seam_axis_state(job, settings.repo_root)
        if axis == 'missing':
            raise ProblemError(409, 'seam-axis-missing', 'seam_axis.json missing',
                               detail='save the seams again to build seam_axis.json (CP-58)')
        if axis == 'stale':
            raise ProblemError(409, 'seam-axis-stale', 'seam_axis.json stale',
                               detail='seams.json, placement.json or frames.yaml changed after seam_axis.json was '
                                      'built; save the seams again, then re-plan')
        subset = body.seams if body else None
        if subset:
            known = {s.get('seam_id') for s in seams_doc.get('seams', [])}
            unknown = [s for s in subset if s not in known]
            if unknown:
                raise ProblemError(422, 'unknown-seam', detail=f'unknown seam id(s): {", ".join(unknown)}',
                                   errors=[{'pointer': '/seams', 'message': f'unknown {s}'} for s in unknown])
        speed = body.travel_speed_mm_s if body else None
        try:
            run = runs.start(job_id, 'plan', lambda d: adapters.plan_steps(settings, d, job, subset, speed),
                             on_success=lambda d: promote_gated(d, job, collision, override))
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id']}

    @api.post('/jobs/{job_id}/program', status_code=202)
    def program(job_id: str, response: Response, body: Optional[ProgramRequest] = None,
                _u: User = Depends(require(*PLAN))) -> Dict[str, str]:
        """D07 → D09 through the program CLI (G1.5) in the run slot; progress like a plan run."""
        job = jobs.job_dir(settings.jobs_dir, job_id)
        state = jobs.document_state(job, 'path.json', settings.repo_root)
        if state == 'missing':
            raise ProblemError(409, 'no-path', 'No path', detail='plan the seams first (path.json, M4)')
        if state == 'stale':
            found = jobs.problems(job, 'path.json', settings.repo_root)
            raise ProblemError(409, 'path-stale', 'path.json stale',
                               detail='path.json is stale; re-plan first: ' + '; '.join(found), problems=found)
        try:
            run = runs.start(job_id, 'program', lambda d: adapters.program_steps(settings, job),
                             on_success=lambda d: adapters.promote_program(d, job))
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id']}

    @api.post('/jobs/{job_id}/replay', status_code=202)
    def replay(job_id: str, response: Response, body: Optional[ReplayRequest] = None,
               _u: User = Depends(require(*PLAN))) -> Dict[str, str]:
        """Replay a current D09 on the user-launched sim_mock (G1.20). Refused before any process starts when the
        program is missing or stale (awm_core.staleness); the CLI itself refuses non-mock hardware."""
        job = jobs.job_dir(settings.jobs_dir, job_id)
        revs = jobs.program_revs(job)
        if not revs:
            raise ProblemError(409, 'no-program', 'No program', detail='generate the program (S4) before replaying it')
        rev = (body.program_rev if body else None) or revs[-1]
        if rev not in revs:
            raise ProblemError(404, 'program-not-found', detail=f'program {rev} not found in job {job_id}')
        name = f'programs/{rev}/program.json'
        state, found = jobs.program_verdict(job, rev, settings.repo_root)  # staleness + void CP-62 override (m3)
        if state == 'missing':
            raise ProblemError(409, 'no-program', 'No program', detail=f'{name} is missing or unreadable')
        if state != 'ok':
            raise ProblemError(409, 'program-stale', f'Program {rev} stale',
                               detail=f'program {rev} is stale; re-plan / generate again before replaying: '
                                      + '; '.join(found), problems=found, program_rev=rev)
        try:
            run = runs.start(job_id, 'replay', replay_mod.replay_steps(settings, job / name))
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id']}

    @api.get('/jobs/{job_id}/simulation')
    def simulation(job_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        """S4 playback data: the newest current program (D09) when there is one, else the path (D07) timing; the
        path (statuses, TCP poses), placement (D19), seams (D04) and the URLs of the view and robot models."""
        job = jobs.job_dir(settings.jobs_dir, job_id)
        path = jobs.read_json(job / 'path.json')
        if path is not None and not str(path.get('schema_version', '')).startswith('2.'):
            path = None  # v1 paths (before G1.3) carry no positioner angle: not playable
        placement = jobs.read_json(job / 'placement.json')
        rev, program, program_state = jobs.latest_program(job, settings.repo_root)
        if placement is None or (path is None and program is None):
            raise ProblemError(404, 'no-simulation', 'No simulation data',
                               detail='needs placement.json and a planned path.json (M4) or a program (M6)')
        notes: List[str] = []
        path_state = jobs.document_state(job, 'path.json', settings.repo_root, path) if path else 'missing'
        if path_state == 'stale':
            notes.append('path.json is stale (seams, placement or cell frames changed): re-plan')
        if program is not None and program_state != 'ok':
            notes.append(f'program {rev} is stale (the path changed after it was generated): generate the program '
                         'again; playing the path timing instead')
        use_program = program is not None and program_state == 'ok'
        path_problems = jobs.problems(job, 'path.json', settings.repo_root, path) if path else []
        program_problems = jobs.program_verdict(job, rev, settings.repo_root, program)[1] if program is not None else []
        checked = [d.get('collision_checked') for d in (path, program if use_program else None) if d is not None]
        return {
            'job_id': job_id, 'source': 'program' if use_program else 'path',
            'program_rev': rev, 'program_state': program_state, 'program': program if use_program else None,
            'path': path, 'path_state': path_state, 'path_problems': path_problems,
            'program_problems': program_problems,
            'replay': replay_mod.info(settings, rev, program_state, program_problems),
            'placement': placement,
            'seams': jobs.read_json(job / 'seams.json'),
            'collision_checked': bool(checked) and all(c is True for c in checked),
            'view_url': f'/api/v1/jobs/{job_id}/view/view.glb',
            'robot_description_url': '/api/v1/cell/robot_description',
            'notes': notes,
        }

    # -- CP-64 jog page kinematics (S11): analytic FK / IK, numeric only, no controller ------------------------------
    @api.post('/cell/fk')
    def cell_fk(body: JogFk, _u: User = Depends(require(*cad_api.CELL_READ))) -> Dict[str, Any]:
        t0 = kin.fk(body.joints_deg)
        tcp = jog.tool0_T_tcp(settings)
        return {'tool0': jog.matrix_to_pose(t0),
                'tcp_laser': None if tcp is None else jog.matrix_to_pose(t0 @ tcp)}

    @api.post('/cell/ik')
    def cell_ik(body: JogIk, _u: User = Depends(require(*cad_api.CELL_WRITE))) -> Dict[str, Any]:
        tcp = jog.tool0_T_tcp(settings)
        if tcp is None:
            raise ProblemError(409, 'ik-no-tcp', 'No taught TCP',
                               detail='head.yaml has no tool0_T_tcp_mm; teach the TCP (S10) first')
        try:
            target = jog.pose_to_matrix(body.target_tcp_laser.model_dump())
            raw = kin.ik(target @ np.linalg.inv(tcp))
        except ValueError as exc:
            raise ProblemError(422, 'validation', detail=str(exc)) from exc
        if not raw:
            raise ProblemError(422, 'ik-unreachable', 'Target out of reach',
                               detail='no joint solution puts the TCP at the target pose')
        ok = [q for q in raw if jog.wrist_singular(q) is None]
        if not ok:
            raise ProblemError(422, 'ik-singular', 'Target on a singularity',
                               detail='every joint solution is within the wrist / elbow / shoulder singularity margin')
        return {'joints_deg': jog.nearest_solution(ok, body.current_joints_deg)}

    @api.get('/cell/home')
    def cell_home_get(_u: User = Depends(require(*cad_api.CELL_READ))) -> Dict[str, Any]:
        try:
            rec = jog.read_home(settings)
        except ValueError as exc:
            raise ProblemError(422, 'home-invalid', 'Home file invalid', detail=str(exc)) from exc
        if rec is None:
            raise ProblemError(404, 'no-home', 'No taught home', detail='cell/model/home.json is missing: teach it in S11')
        return rec

    @api.put('/cell/home')
    def cell_home_put(body: HomeBody, u: User = Depends(require(*cad_api.CELL_WRITE))) -> Dict[str, Any]:
        try:
            return jog.write_home(settings, u.username, body.robot_joints_deg, body.positioner_joint_deg)
        except cad_api.jsonio.SchemaValidationError as exc:
            raise ProblemError(422, 'validation', 'Home invalid', detail=str(exc)) from exc
        except FileNotFoundError as exc:
            raise ProblemError(409, 'no-frames', 'No cell frames',
                               detail='cell/model/frames.yaml is missing: export the cell scene (S10) first') from exc

    # -- cell model for the browser (display only) ------------------------------------------------------------------
    @api.get('/cell/robot_description')
    def robot_description(_u: User = Depends(require(*READ))) -> Dict[str, Any]:
        return robot.description()

    @api.get('/cell/meshes/head/{path:path}')
    def head_mesh(path: str, _u: User = Depends(require(*READ))) -> FileResponse:
        return FileResponse(robot.head_mesh(path), media_type='application/octet-stream')

    def scene_manifest() -> Dict[str, Any]:
        doc = jobs.read_json(settings.cell_model_dir / 'scene_manifest.json')
        if doc is None or not isinstance(doc.get('objects'), list):
            raise ProblemError(404, 'no-scene', 'No cell scene',
                               detail='cell/model/scene_manifest.json is missing: save the cell model in S10 (CP-60)')
        return doc

    @api.get('/cell/scene')
    def cell_scene(_u: User = Depends(require(*READ))) -> Dict[str, Any]:
        """S4 cell view (display only): the export-scene objects with their parent frame (``world`` = static,
        ``positioner_flange`` = turns with θ), pose (mm, θ = 0) and a mesh URL. No geometry is changed here."""
        doc = scene_manifest()
        objects = []
        for o in doc['objects']:
            mesh = (o.get('mesh') or {}).get('file')
            if not isinstance(mesh, str):
                continue
            sha = (o.get('mesh') or {}).get('sha256')
            # review m5: the manifest hash in the URL keys the browser cache to this exact mesh
            query = f'?sha={sha}' if isinstance(sha, str) and re.fullmatch(r'[0-9a-f]{64}', sha) else ''
            objects.append({k: o.get(k) for k in ('object_id', 'role', 'component', 'parent_frame', 'pose_mm',
                                                   'collision')}
                           | {'mesh_url': f'{API_MESHES}/scene/{mesh}{query}'})
        return {'unit': doc.get('unit', 'mm'), 'created_at': doc.get('created_at'), 'note': doc.get('note'),
                'objects': objects}

    @api.get('/cell/meshes/scene/{path:path}')
    def scene_mesh(path: str, _u: User = Depends(require(*READ))) -> FileResponse:
        listed = {(o.get('mesh') or {}).get('file') for o in scene_manifest()['objects']}
        if path not in listed:
            raise ProblemError(403, 'forbidden-path', detail=f'{path!r} is not a mesh of the cell scene manifest')
        return FileResponse(safe_child(settings.cell_model_dir, path), media_type='application/octet-stream')

    @api.get('/cell/meshes/package/{package}/{path:path}')
    def package_mesh(package: str, path: str, _u: User = Depends(require(*READ))) -> FileResponse:
        return FileResponse(robot.package_mesh(package, path), media_type='application/octet-stream')

    # -- runs -----------------------------------------------------------------------------------------------------
    def _run_or_404(run_id: str) -> Dict[str, Any]:
        try:
            return runs.get(run_id)
        except RunNotFoundError:
            raise ProblemError(404, 'run-not-found', detail=f'run {run_id} not found') from None

    @api.get('/runs/{run_id}')
    def get_run(run_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        return _run_or_404(run_id)

    @api.delete('/runs/{run_id}', status_code=202)
    def cancel_run(run_id: str, _u: User = Depends(require(*CANCEL))) -> Dict[str, Any]:
        _run_or_404(run_id)
        try:
            return runs.cancel(run_id)
        except RunFinishedError as exc:
            raise ProblemError(409, 'run-finished', detail=str(exc)) from exc

    @api.websocket('/runs/{run_id}/ws')
    async def run_ws(ws: WebSocket, run_id: str) -> None:
        """Progress push (G1.4 addition to the polled contract): the run document on every change, ≤ 10 Hz."""
        if tokens.user(ws.query_params.get('token')) is None:
            await ws.close(code=4401)
            return
        try:
            doc = runs.get(run_id)
        except RunNotFoundError:
            await ws.close(code=4404)
            return
        await ws.accept()
        last = None
        try:
            while True:
                doc = runs.get(run_id)
                if doc != last:
                    await ws.send_json(doc)
                    last = doc
                if doc['state'] in TERMINAL:
                    break
                await asyncio.sleep(0.1)
            await ws.close()
        except WebSocketDisconnect:
            pass

    cad_api.register(api, settings, runs, require, _busy, admins)
    app.include_router(api)
    _mount_front_end(app, settings)
    return app


def _mount_front_end(app: FastAPI, settings: Settings) -> None:
    """Serve the built ``awm_web`` (``dist/``) with an index.html fallback; /api/* stays problem+json."""
    dist = settings.web_dist
    if dist is None:
        return
    dist = dist.resolve()
    index = dist / 'index.html'
    if not index.is_file():
        raise FileNotFoundError(f'{index} not found; build the front-end (npm run build) first')

    @app.get('/{path:path}', include_in_schema=False)
    def front_end(path: str, request: Request):  # noqa: ANN202
        if path == 'api' or path.startswith('api/'):
            raise ProblemError(404, 'not-found', detail=f'{request.url.path} is not an API endpoint')
        candidate = (dist / path).resolve()
        if path and dist in candidate.parents and candidate.is_file():
            return FileResponse(candidate)
        return FileResponse(index, headers={'Cache-Control': 'no-store'})
