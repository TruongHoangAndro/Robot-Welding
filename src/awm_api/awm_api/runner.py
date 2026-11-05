"""Planning-run runner (PLAN.md T2.7, B-11): engineering CLIs as subprocesses, one run slot per cell.

* Every run has a folder ``<runs_dir>/<run_id>/`` with ``run.json`` (the state served by ``GET /runs/{id}``),
  ``log.txt`` (merged stdout / stderr of all steps) and the run inputs written by the caller.
* One slot per cell: a second run (or a synchronous slot holder such as a seam save) while one is active raises
  :class:`RunBusyError` (HTTP 409). The run is the job lock.
* Cancel = SIGTERM to the process group of the running step; SIGKILL only if it is still alive after
  ``kill_grace_s``. Outputs of a cancelled or failed run are never promoted (``on_success`` is not called).
* Progress: a step may print lines ``progress <0..1> <text>`` (also ``progress: …``) on stdout or stderr; the run
  progress is the weighted sum over the steps and never decreases.

No rclpy, no OCC here: the runner only starts processes.
"""

from __future__ import annotations

import collections
import contextlib
import datetime as _dt
import json
import logging
import os
import re
import secrets
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, List, Optional, Sequence, Tuple, Union

LOG = logging.getLogger('awm_api.runner')

TERMINAL = ('succeeded', 'failed', 'cancelled')
RUN_ID_RE = re.compile(r'^R[0-9]{8}T[0-9]{6}Z-[0-9a-f]{6}$')
_PROGRESS_RE = re.compile(r'\bprogress:?\s+([0-9]*\.?[0-9]+)\s*(.*)$', re.IGNORECASE)
_SAVE_INTERVAL_S = 0.2


def utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')


def new_run_id() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime('R%Y%m%dT%H%M%SZ-') + secrets.token_hex(3)


def parse_progress(line: str) -> Optional[Tuple[float, str]]:
    """``progress 0.42 text`` → (0.42, 'text'); None if the line carries no valid fraction."""
    m = _PROGRESS_RE.search(line)
    if not m:
        return None
    frac = float(m.group(1))
    if not 0.0 <= frac <= 1.0:
        return None
    return frac, m.group(2).strip()


@dataclass
class Step:
    """One subprocess of a run."""

    label: str
    argv: List[str]
    env: Optional[Dict[str, str]] = None
    cwd: Optional[Path] = None
    weight: float = 1.0


class RunBusyError(RuntimeError):
    def __init__(self, active_run: str) -> None:
        super().__init__(f'another run is active: {active_run}')
        self.active_run = active_run


class RunNotFoundError(KeyError):
    pass


class RunFinishedError(RuntimeError):
    pass


class StepsError(RuntimeError):
    """Raised by a step builder to refuse a run before any process starts (message for the user)."""


StepsArg = Union[Sequence[Step], Callable[[Path], Sequence[Step]]]
Hook = Optional[Callable[[Path], Any]]


class _Run:
    def __init__(self, run_id: str, job_id: str, kind: str, run_dir: Path) -> None:
        self.dir = run_dir
        self.doc: Dict[str, Any] = {
            'run_id': run_id, 'job_id': job_id, 'kind': kind, 'state': 'queued', 'progress': 0.0,
            'step': 'queued', 'started_at': utc_now(), 'finished_at': None, 'log_tail': [], 'outputs': {},
            'problem': None}
        self.tail: collections.deque = collections.deque(maxlen=50)
        self.cancel_requested = False
        self.proc: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()
        self.last_save = 0.0
        self.version = 0

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            doc = dict(self.doc)
            doc['log_tail'] = list(self.tail)
            doc['outputs'] = dict(self.doc['outputs'])
            return doc

    def update(self, force: bool = False, **fields: Any) -> None:
        with self.lock:
            self.doc.update(fields)
            self.version += 1
        now = time.monotonic()
        if force or now - self.last_save >= _SAVE_INTERVAL_S:
            self.last_save = now
            self.save()

    def save(self) -> None:
        doc = self.snapshot()
        tmp = self.dir / 'run.json.tmp'
        tmp.write_text(json.dumps(doc, indent=1) + '\n')
        tmp.replace(self.dir / 'run.json')


class RunManager:
    """Runs and the single run slot of one cell."""

    def __init__(self, runs_dir: Path, kill_grace_s: float = 10.0) -> None:
        self.runs_dir = Path(runs_dir)
        self.kill_grace_s = kill_grace_s
        self._runs: Dict[str, _Run] = {}
        self._lock = threading.Lock()
        self._active: Optional[Dict[str, str]] = None
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self._fail_orphans()

    # -- slot ---------------------------------------------------------------------------------------------------
    def _acquire(self, holder: Dict[str, str]) -> None:
        with self._lock:
            if self._active is not None:
                raise RunBusyError(self._active['run_id'])
            self._active = holder

    def _release(self, run_id: str) -> None:
        with self._lock:
            if self._active and self._active['run_id'] == run_id:
                self._active = None

    def active(self) -> Optional[Dict[str, str]]:
        """``{'run_id', 'job_id', 'kind'}`` of the slot holder, or None."""
        with self._lock:
            return dict(self._active) if self._active else None

    def active_run(self) -> Optional[str]:
        a = self.active()
        return a['run_id'] if a else None

    @contextlib.contextmanager
    def slot(self, kind: str, job_id: str) -> Iterator[None]:
        """Hold the cell slot for a short synchronous operation (e.g. a seam save)."""
        holder = {'run_id': f'{kind}:{job_id}:{secrets.token_hex(3)}', 'job_id': job_id, 'kind': kind}
        self._acquire(holder)
        try:
            yield
        finally:
            self._release(holder['run_id'])

    # -- runs ---------------------------------------------------------------------------------------------------
    def start(self, job_id: str, kind: str, steps: StepsArg, on_success: Hook = None,
              on_failure: Hook = None) -> Dict[str, Any]:
        """Start a run in a background thread. ``steps`` may be a builder called with the run folder."""
        run_id = new_run_id()
        self._acquire({'run_id': run_id, 'job_id': job_id, 'kind': kind})
        run_dir = self.runs_dir / run_id
        try:
            run_dir.mkdir(parents=True)
            step_list = list(steps(run_dir) if callable(steps) else steps)
            run = _Run(run_id, job_id, kind, run_dir)
            run.save()
        except BaseException:
            self._release(run_id)
            shutil.rmtree(run_dir, ignore_errors=True)
            raise
        with self._lock:
            self._runs[run_id] = run
        threading.Thread(target=self._execute, args=(run, step_list, on_success, on_failure),
                         name=f'run-{run_id}', daemon=True).start()
        LOG.info('run %s started: %s %s (%d step(s))', run_id, kind, job_id, len(step_list))
        return run.snapshot()

    def get(self, run_id: str) -> Dict[str, Any]:
        with self._lock:
            run = self._runs.get(run_id)
        if run is not None:
            return run.snapshot()
        if not RUN_ID_RE.match(run_id):
            raise RunNotFoundError(run_id)
        path = self.runs_dir / run_id / 'run.json'
        if not path.is_file():
            raise RunNotFoundError(run_id)
        return json.loads(path.read_text())

    def version(self, run_id: str) -> int:
        with self._lock:
            run = self._runs.get(run_id)
        return run.version if run else -1

    def runs_for(self, kind: str) -> List[Dict[str, Any]]:
        with self._lock:
            runs = list(self._runs.values())
        return [r.snapshot() for r in runs if r.doc['kind'] == kind]

    def cancel(self, run_id: str) -> Dict[str, Any]:
        with self._lock:
            run = self._runs.get(run_id)
        if run is None:
            doc = self.get(run_id)  # raises RunNotFoundError
            raise RunFinishedError(f'run {run_id} is {doc["state"]}')
        with run.lock:
            if run.doc['state'] in TERMINAL:
                raise RunFinishedError(f'run {run_id} is {run.doc["state"]}')
            run.cancel_requested = True
            proc = run.proc
        LOG.info('run %s: cancel requested', run_id)
        if proc is not None:
            self._terminate(proc)
        return run.snapshot()

    def shutdown(self) -> None:
        """Cancel the active run (API shutdown)."""
        rid = self.active_run()
        if rid and rid in self._runs:
            with contextlib.suppress(RunFinishedError, RunNotFoundError):
                self.cancel(rid)

    # -- internals ----------------------------------------------------------------------------------------------
    def _terminate(self, proc: subprocess.Popen) -> None:
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(proc.pid, signal.SIGTERM)

        def _kill_later() -> None:
            try:
                proc.wait(timeout=self.kill_grace_s)
            except subprocess.TimeoutExpired:
                LOG.warning('process %d ignored SIGTERM for %.0f s; SIGKILL', proc.pid, self.kill_grace_s)
                with contextlib.suppress(ProcessLookupError, PermissionError):
                    os.killpg(proc.pid, signal.SIGKILL)

        threading.Thread(target=_kill_later, daemon=True).start()

    def _fail_orphans(self) -> None:
        for path in self.runs_dir.glob('R*/run.json'):
            try:
                doc = json.loads(path.read_text())
            except ValueError:
                continue
            if doc.get('state') in ('queued', 'running'):
                doc.update(state='failed', finished_at=utc_now(), problem={
                    'type': '/problems/run-orphaned', 'title': 'Run lost', 'status': 500,
                    'detail': 'awm_api restarted while the run was active; outputs were not promoted'})
                path.write_text(json.dumps(doc, indent=1) + '\n')

    def _execute(self, run: _Run, steps: List[Step], on_success: Hook, on_failure: Hook) -> None:
        total = sum(max(s.weight, 0.0) for s in steps) or 1.0
        done_w = 0.0
        log = (run.dir / 'log.txt').open('a', buffering=1)
        try:
            for step in steps:
                if run.cancel_requested:
                    break
                run.update(force=True, state='running', step=step.label)
                rc, error = self._run_step(run, step, log, done_w, total)
                if run.cancel_requested:
                    break
                if error is not None or rc != 0:
                    detail = error or self._failure_detail(run, step, rc)
                    self._finish(run, 'failed', on_failure, problem={
                        'type': '/problems/run-failed', 'title': 'Run failed', 'status': 422, 'detail': detail})
                    return
                done_w += max(step.weight, 0.0)
                run.update(progress=max(run.doc['progress'], done_w / total))
            if run.cancel_requested:
                self._finish(run, 'cancelled', on_failure, step='cancelled')
                return
            try:
                outputs = on_success(run.dir) if on_success else {}
            except Exception as exc:  # noqa: BLE001 - promotion failed: report it as the run result
                LOG.exception('run %s: promoting outputs failed', run.doc['run_id'])
                self._finish(run, 'failed', on_failure, problem={
                    'type': '/problems/run-failed', 'title': 'Run failed', 'status': 422,
                    'detail': f'outputs could not be stored: {exc}'})
                return
            run.update(outputs=outputs or {})
            self._finish(run, 'succeeded', None, progress=1.0, step='done')
        finally:
            log.close()

    def _finish(self, run: _Run, state: str, hook: Hook, **fields: Any) -> None:
        if hook is not None:
            try:
                hook(run.dir)
            except Exception:  # noqa: BLE001 - cleanup must not hide the run result
                LOG.exception('run %s: cleanup hook failed', run.doc['run_id'])
        run.update(force=True, state=state, finished_at=utc_now(), **fields)
        self._release(run.doc['run_id'])
        LOG.info('run %s %s', run.doc['run_id'], state)

    def _run_step(self, run: _Run, step: Step, log, done_w: float, total: float) -> Tuple[int, Optional[str]]:
        log.write(f'=== {utc_now()} {step.label}: {" ".join(step.argv)}\n')
        try:
            proc = subprocess.Popen(step.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                    bufsize=1, env=step.env, cwd=step.cwd, start_new_session=True)
        except OSError as exc:
            return -1, f'{step.label}: cannot start {step.argv[0]!r}: {exc}'
        with run.lock:
            run.proc = proc
            cancel_now = run.cancel_requested
        if cancel_now:
            self._terminate(proc)
        assert proc.stdout is not None
        for raw in proc.stdout:
            line = raw.rstrip('\n')
            log.write(line + '\n')
            with run.lock:
                run.tail.append(line[:500])
            p = parse_progress(line)
            if p is not None:
                frac, text = p
                overall = (done_w + max(step.weight, 0.0) * frac) / total
                run.update(progress=max(run.doc['progress'], min(overall, 1.0)),
                           step=f'{step.label}: {text}' if text else step.label)
            else:
                run.update()
        rc = proc.wait()
        with run.lock:
            run.proc = None
        log.write(f'=== {step.label} exit {rc}\n')
        return rc, None

    @staticmethod
    def _failure_detail(run: _Run, step: Step, rc: int) -> str:
        with run.lock:
            lines = list(run.tail)
        errors = [ln for ln in lines if re.search(r'error|exception|refus|fail', ln, re.IGNORECASE)]
        shown = (errors or lines)[-5:]
        return f'{step.label} failed (exit {rc})' + (': ' + ' | '.join(shown) if shown else '')
