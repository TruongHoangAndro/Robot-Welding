"""FastAPI application: ``/api/v1`` (contract docs/generated/contracts/awm_api_sim1.openapi.yaml) + static front-end.

Roles (DES-001 §7.3, §4.5 S1 / S2, CP-21): every role reads; jobs are created by Engineer or Admin (S1); seams
are edited by Engineer only (CP-21); planning runs are started by Engineer (§7.3 "run simulation"); runs are
cancelled by Engineer or Admin. Nothing here commands motion, the PLC or the laser.
"""

from __future__ import annotations

import asyncio
import contextlib
import datetime as _dt
import json
import logging
import re
import secrets
from pathlib import Path
from typing import Annotated, Any, Dict, List, Literal, Optional

from fastapi import APIRouter, Depends, FastAPI, File, Form, Header, Request, Response, UploadFile, WebSocket
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, ConfigDict, Field
from starlette.websockets import WebSocketDisconnect

from awm_api import adapters, jobs
from awm_api.auth import TokenStore, User, forbidden, load_users, unauthorized, verify_password
from awm_api.problems import ProblemError, install as install_problems
from awm_api.runner import TERMINAL, RunBusyError, RunFinishedError, RunManager, RunNotFoundError, StepsError
from awm_api.settings import Settings

LOG = logging.getLogger('awm_api')

READ = ('Operator', 'Engineer', 'Admin')
CREATE_JOB = ('Engineer', 'Admin')
EDIT_SEAMS = ('Engineer',)
PLAN = ('Engineer',)
CANCEL = ('Engineer', 'Admin')
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')
STEP_SUFFIXES = ('.step', '.stp')

SeamId = Annotated[str, Field(pattern=r'^S[0-9]{2,3}$')]
EdgeId = Annotated[str, Field(pattern=r'^E[0-9]+$')]


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
    model_config = ConfigDict(extra='forbid')
    seams: Optional[List[SeamId]] = None


def _check_picks(picks: List[SeamPick]) -> None:
    errors = []
    seen = set()
    for i, p in enumerate(picks):
        if p.seam_id in seen:
            errors.append({'pointer': f'/seams/{i}/seam_id', 'message': f'duplicate seam id {p.seam_id}'})
        seen.add(p.seam_id)
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
    tokens = TokenStore(settings.token_ttl_s)
    runs = RunManager(settings.runs_dir, kill_grace_s=settings.kill_grace_s)
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
        def dep(user: User = Depends(current_user)) -> User:
            if user.role not in roles:
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
            return adapters.import_steps(settings, run_dir, target, part_no, part_rev, component or None, up_axis)

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
            raise ProblemError(404, 'no-placement', 'No placement', detail=jobs.NO_PLACEMENT_NOTE)
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
        has_placement = (job / 'placement.json').is_file()
        stamp = _dt.datetime.now(_dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        work = settings.repo_root / 'logs' / 'seam_saves' / f'{job_id}-{stamp}-{secrets.token_hex(3)}'
        picks = [p.model_dump() for p in body.seams]
        try:
            with runs.slot('seams', job_id):
                doc = adapters.build_seams(settings, job, picks, work, build_axis=has_placement)
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['X-AWM-Seam-Axis'] = (
            'built' if has_placement else 'not built: no placement.json (Goal 1 placement comes from the cell CAD)')
        return doc

    @api.post('/jobs/{job_id}/plan', status_code=202)
    def plan(job_id: str, response: Response, body: Optional[PlanRequest] = None,
             _u: User = Depends(require(*PLAN))) -> Dict[str, str]:
        job = jobs.job_dir(settings.jobs_dir, job_id)
        seams_doc = jobs.read_json(job / 'seams.json')
        if seams_doc is None:
            raise ProblemError(409, 'no-seams', 'No seams', detail='save the seams (S2) before planning')
        if not (job / 'placement.json').is_file():
            raise ProblemError(409, 'no-placement', 'No placement', detail=jobs.NO_PLACEMENT_NOTE)
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
        try:
            run = runs.start(job_id, 'plan', lambda d: adapters.plan_steps(settings, d, job, subset),
                             on_success=lambda d: adapters.promote_plan(d, job))
        except RunBusyError as exc:
            raise _busy(exc) from exc
        response.headers['Location'] = f'/api/v1/runs/{run["run_id"]}'
        return {'run_id': run['run_id']}

    @api.get('/jobs/{job_id}/simulation')
    def simulation(job_id: str, _u: User = Depends(require(*READ))) -> Dict[str, Any]:
        """S4 playback data; available once a v2 path.json exists (G1.3 / G1.5)."""
        job = jobs.job_dir(settings.jobs_dir, job_id)
        path = jobs.read_json(job / 'path.json')
        placement = jobs.read_json(job / 'placement.json')
        if not path or not str(path.get('schema_version', '')).startswith('2.') or placement is None:
            raise ProblemError(404, 'no-simulation', 'No simulation data',
                               detail='no v2 path.json with a placement yet (planner G1.3 / program G1.5)')
        return {'path': path, 'placement': placement}

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
