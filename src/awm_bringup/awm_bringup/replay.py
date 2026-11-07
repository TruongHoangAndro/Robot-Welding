"""Pure core of the D09 replay (G1.7): program checks, start-state check, deg -> rad, summaries. No rclpy.

A D09 program v2 (MINOR 2.1, ``awm_program.cli generate``, G1.5) holds one ``robot_positioner`` trajectory per
segment: positions / velocities / accelerations in the order ``joint_names.robot`` then ``joint_names.positioner``,
in degrees and seconds. The replay sends each segment unchanged (no time scaling: nothing runs faster than
planned) to the single 7-joint controller of ``sim_mock`` (DES-001 §6.3 Option A1). Radians only through
``awm_core.units`` (ADR-14).

Refusals (:class:`ReplayRefused`): not a 2.x program, a segment of another controller, a point without 7 values,
a value that is not finite (NaN / inf), a segment that does not start where the previous one ends or does not end at rest,
a controller action other than the 7-joint controller, a hardware component that is not mock hardware. Before every segment the
current joint state must be finite and equal the segment start within ``start_state_tolerance_deg`` (:class:`StartStateMismatch`,
DES-001 §4.2 M9 start-state check). A refusal after the
first segment leaves the earlier segments executed: status ``partial``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple, Union

import yaml

from awm_core import units

CONTROLLER = 'robot_positioner'
# The only action the replay sends to (sim_mock trajectory_controllers:=robot_positioner, DES-001 §6.3 Option A1).
REQUIRED_ACTION = '/robot_positioner_trajectory_controller/follow_joint_trajectory'
# Hardware plugins accepted by the replay (ADR-13: mock hardware only; UR mock and the awm_positioner mock).
MOCK_PLUGINS = ('mock_components/GenericSystem',)
SCHEMA = 'v2/program'
CONTINUITY_TOL_DEG = 1e-6
REST_TOL_DEG_S = 1e-6


class ReplayRefused(Exception):
    """The program cannot be replayed; nothing is sent."""


class StartStateMismatch(ReplayRefused):
    """The current joint state is not the segment start; the segment is not sent."""


@dataclass(frozen=True)
class ReplayConfig:
    """``program_replay`` section of ``awm_bringup/config/program_replay.yaml`` (D16)."""

    controller_action: str
    start_state_tolerance_deg: float
    final_tolerance_deg: float
    wait_timeout_s: float
    result_margin_s: float
    settle_s: float


def default_config_path() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_bringup')) / 'config' / 'program_replay.yaml'


def load_config(path: Optional[str]) -> ReplayConfig:
    """Read the replay section; every number must be > 0 (tolerances, timeouts)."""
    src = Path(path) if path else default_config_path()
    sec = (yaml.safe_load(src.read_text()) or {}).get('program_replay', {})
    defaults = yaml.safe_load(default_config_path().read_text())['program_replay'] if path else {}
    merged = {**defaults, **sec}
    try:
        cfg = ReplayConfig(str(merged['controller_action']), float(merged['start_state_tolerance_deg']),
                           float(merged['final_tolerance_deg']), float(merged['wait_timeout_s']),
                           float(merged['result_margin_s']), float(merged['settle_s']))
    except KeyError as exc:
        raise ValueError(f'{src}: program_replay.{exc.args[0]} missing') from exc
    if cfg.controller_action != REQUIRED_ACTION:
        raise ReplayRefused(f'{src}: controller_action {cfg.controller_action!r}; the replay sends only to '
                            f'{REQUIRED_ACTION} (mock 7-joint controller of sim_mock)')
    for name in ('start_state_tolerance_deg', 'final_tolerance_deg', 'wait_timeout_s', 'result_margin_s'):
        if not getattr(cfg, name) > 0.0:
            raise ValueError(f'{src}: program_replay.{name} must be > 0')
    if cfg.settle_s < 0.0:
        raise ValueError(f'{src}: program_replay.settle_s must be >= 0')
    return cfg


@dataclass(frozen=True)
class ReplaySegment:
    segment_id: str
    kind: str
    seam_id: Optional[str]
    times_s: Tuple[float, ...]
    positions_deg: Tuple[Tuple[float, ...], ...]
    velocities_deg_s: Tuple[Tuple[float, ...], ...]
    accelerations_deg_s2: Tuple[Tuple[float, ...], ...]

    @property
    def start_deg(self) -> Tuple[float, ...]:
        return self.positions_deg[0]

    @property
    def end_deg(self) -> Tuple[float, ...]:
        return self.positions_deg[-1]

    @property
    def duration_s(self) -> float:
        return self.times_s[-1]

    def trajectory_rad(self) -> Tuple[List[float], List[List[float]], List[List[float]], List[List[float]]]:
        """Times (s) and positions / velocities / accelerations in rad, rad/s, rad/s² for the controller."""
        conv = lambda rows: [[float(v) for v in units.deg_to_rad(list(r))] for r in rows]  # noqa: E731
        return (list(self.times_s), conv(self.positions_deg), conv(self.velocities_deg_s),
                conv(self.accelerations_deg_s2))


@dataclass(frozen=True)
class ReplayProgram:
    program_rev: str
    joint_names: List[str]
    segments: List[ReplaySegment]

    @classmethod
    def from_doc(cls, doc: Mapping) -> 'ReplayProgram':
        ver = str(doc.get('schema_version', ''))
        if not ver.startswith('2.'):
            raise ReplayRefused(f'schema_version {ver!r}: only D09 v2 programs (MINOR >= 2.1) can be replayed')
        names = list(doc['joint_names']['robot']) + list(doc['joint_names']['positioner'])
        n = len(names)
        if n != 7:
            raise ReplayRefused(f'program has {n} joints; the 7-joint controller needs 6 robot + 1 positioner')
        segs: List[ReplaySegment] = []
        for raw in doc['segments']:
            sid = raw['segment_id']
            if raw['controller'] != CONTROLLER:
                raise ReplayRefused(f'{sid}: controller {raw["controller"]!r}; this replay sends only '
                                    f'{CONTROLLER!r} segments (one 7-joint trajectory)')
            pts = raw['points']
            for k, p in enumerate(pts):
                if not _finite([p['time_from_start_s']]):
                    raise ReplayRefused(f'{sid} point {k}: time_from_start_s is not finite')
                for key in ('positions_deg', 'velocities_deg_s', 'accelerations_deg_s2'):
                    if len(p[key]) != n:
                        raise ReplayRefused(f'{sid} point {k}: {key} needs {n} values, has {len(p[key])}')
                    if not _finite(p[key]):
                        raise ReplayRefused(f'{sid} point {k}: {key} has a value that is not finite')
            times = tuple(float(p['time_from_start_s']) for p in pts)
            if any(b <= a for a, b in zip(times, times[1:])):
                raise ReplayRefused(f'{sid}: time_from_start_s not strictly increasing')
            seg = ReplaySegment(sid, raw['kind'], raw.get('seam_id'), times,
                                tuple(tuple(float(v) for v in p['positions_deg']) for p in pts),
                                tuple(tuple(float(v) for v in p['velocities_deg_s']) for p in pts),
                                tuple(tuple(float(v) for v in p['accelerations_deg_s2']) for p in pts))
            for edge, vel in (('start', seg.velocities_deg_s[0]), ('end', seg.velocities_deg_s[-1])):
                if max(abs(v) for v in vel) > REST_TOL_DEG_S:
                    raise ReplayRefused(f'{sid}: {edge} velocity is not at rest (segments start and end at rest)')
            if segs and max(abs(a - b) for a, b in zip(segs[-1].end_deg, seg.start_deg)) > CONTINUITY_TOL_DEG:
                raise ReplayRefused(f'{sid} does not start where {segs[-1].segment_id} ends')
            segs.append(seg)
        if not segs:
            raise ReplayRefused('program has no segments')
        return cls(str(doc.get('program_rev', '?')), names, segs)

    def select(self, spec: Optional[str]) -> List[ReplaySegment]:
        """``None`` = all; ``G003`` = one; ``G002-G004`` = inclusive range (program order)."""
        if not spec:
            return list(self.segments)
        ids = [s.segment_id for s in self.segments]
        first, _, last = spec.partition('-')
        last = last or first
        for sid in (first, last):
            if sid not in ids:
                raise ReplayRefused(f'segment {sid} not in program {self.program_rev} ({ids[0]}…{ids[-1]})')
        a, b = ids.index(first), ids.index(last)
        if b < a:
            raise ReplayRefused(f'segment range {spec} is reversed')
        return self.segments[a:b + 1]


def load_program(path: Union[str, Path], validate_schema: bool = True) -> ReplayProgram:
    """Read a D09 file; with ``validate_schema`` it must match ``v2/program.schema.json`` first."""
    doc = json.loads(Path(path).read_text())
    if validate_schema:
        from awm_core import jsonio
        try:
            jsonio.validate(doc, SCHEMA)
        except Exception as exc:  # SchemaValidationError lists all errors
            raise ReplayRefused(f'{path}: not a valid D09 v2 program (schema): {exc}') from exc
    return ReplayProgram.from_doc(doc)


def _finite(values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def check_mock_hardware(components: Sequence[Tuple[str, str]]) -> None:
    """``(name, plugin)`` of every ros2_control hardware component; refuse unless all are mock hardware."""
    if not components:
        raise ReplayRefused('no hardware component reported by the controller manager: cannot prove mock hardware')
    bad = [(n, p) for n, p in components if p not in MOCK_PLUGINS]
    if bad:
        raise ReplayRefused('replay runs on mock hardware only (ADR-13); not mock: '
                            + ', '.join(f'{n} ({p or "plugin unknown"})' for n, p in bad))


def refusal_status(segments_done: int) -> str:
    """``refused`` when nothing moved, ``partial`` when earlier segments were already executed."""
    return 'partial' if segments_done > 0 else 'refused'


def _deviations(expected_deg: Sequence[float], names: Sequence[str], current_deg: Mapping[str, float]
                ) -> List[Tuple[str, float]]:
    missing = [j for j in names if j not in current_deg]
    if missing:
        raise StartStateMismatch(f'joint state has no {", ".join(missing)}')
    bad = [j for j in names if not _finite([current_deg[j]])]
    if bad:
        raise StartStateMismatch(f'joint state {", ".join(f"{j} = {current_deg[j]}" for j in bad)} is not finite')
    return [(j, abs(float(current_deg[j]) - float(e))) for j, e in zip(names, expected_deg)]


def deviation_deg(expected_deg: Sequence[float], names: Sequence[str], current_deg: Mapping[str, float]) -> float:
    """Largest |current − expected| over the joints (deg)."""
    return max(d for _, d in _deviations(expected_deg, names, current_deg))


def check_start_state(seg: ReplaySegment, names: Sequence[str], current_deg: Mapping[str, float],
                      tol_deg: float) -> float:
    """Return the largest start deviation (deg) or raise :class:`StartStateMismatch` naming the joints."""
    devs = _deviations(seg.start_deg, names, current_deg)
    bad = [(j, d) for j, d in devs if d > tol_deg]
    if bad:
        txt = ', '.join(f'{j} {current_deg[j]:.4f} deg (segment start {seg.start_deg[names.index(j)]:.4f}, '
                        f'off {d:.4f})' for j, d in bad)
        raise StartStateMismatch(f'{seg.segment_id}: current state is not the segment start '
                                 f'(tolerance {tol_deg} deg): {txt}')
    return max(d for _, d in devs)


class TrackingStats:
    """Maximum |desired − actual| over all joints from controller feedback (input rad, result deg)."""

    def __init__(self) -> None:
        self.max_deg = 0.0
        self.samples = 0

    def add(self, error_rad: Sequence[float]) -> None:
        if len(error_rad):
            self.max_deg = max(self.max_deg, max(abs(float(v)) for v in units.rad_to_deg(list(error_rad))))
        self.samples += 1


def positions_deg(names: Sequence[str], positions_rad: Sequence[float]) -> Dict[str, float]:
    """``/joint_states`` (rad) → {joint: deg}."""
    return dict(zip(names, (float(v) for v in units.rad_to_deg(list(positions_rad)))))
