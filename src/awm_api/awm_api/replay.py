"""S4 "Replay in RViz" (G1.20, close-out B1): the replay run and its user messages. No rclpy (B-11).

``awm_api`` starts ``ros2 run awm_bringup replay_program --program <job>/programs/<rev>/program.json`` (G1.7) as a
run against the ``sim_mock`` the user launched (``Settings.replay_launch_line``). The CLI is the only process that
talks to the controller; it checks the controller manager for mock hardware only and refuses anything else (exit 2,
nothing sent). This module adds no option that could change that: the command line carries ``--program`` only (no
config, no segment range). The CLI's last line ``replay {json}`` becomes the run ``result``; a failed run gets a
problem whose ``detail`` says what to do (:func:`explain`).

Weld events of the program are never executed by the replay (motion only; no PLC, no laser; DES-001 ADR-04).
"""

from __future__ import annotations

import functools
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

from awm_api.runner import Step
from awm_api.settings import Settings

RESULT_PREFIX = 'replay'
CONTROLLER = '/robot_positioner_trajectory_controller'


def _cause(reason: str) -> str:
    r = reason.lower()
    if 'list_hardware_components' in r:
        return 'sim-not-running'
    if 'trajectory_controllers:=robot_positioner' in r or 'follow_joint_trajectory not available' in r:
        return 'controller-absent'
    if 'mock hardware only' in r or 'cannot prove mock hardware' in r or 'no hardware component' in r:
        return 'not-mock'
    if 'not the segment start' in r or 'joint state has no' in r or 'is not finite' in r:
        return 'start-state'
    return 'error'


def explain(rc: int, result: Optional[Dict[str, Any]], tail: List[str], launch: str,
            domain: Optional[str]) -> Dict[str, Any]:
    """Problem (RFC 9457 members + ``cause``, ``replay_status``) for a failed replay run."""
    refused = rc == 2
    base = {'status': 409 if refused else 422, 'title': 'Replay refused' if refused else 'Replay failed'}
    if not result:
        shown = ' | '.join([ln for ln in tail if ln.strip()][-4:])
        return {**base, 'type': '/problems/replay-no-result', 'cause': 'no-result', 'replay_status': None,
                'detail': f'replay_program ended without a result line (exit {rc}); is awm_bringup built and sourced '
                          f'where awm_api runs?' + (f' Last output: {shown}' if shown else '')}
    status = result.get('status')
    reason = str(result.get('reason') or f'exit {rc}')
    cause = _cause(reason)
    done = len(result.get('segments') or [])
    dom = f' on ROS_DOMAIN_ID {domain}' if domain else ''
    if cause == 'sim-not-running':
        detail = (f'sim_mock is not running (no controller manager answered{dom}). Launch it in a terminal of the '
                  f'same container and ROS domain, then press Replay again: {launch}')
    elif cause == 'controller-absent':
        detail = (f'The 7-joint controller {CONTROLLER} is not running: sim_mock was launched without '
                  f'trajectory_controllers:=robot_positioner. Stop it and launch: {launch}')
    elif cause == 'not-mock':
        detail = (f'Refused: the controller manager reports hardware that is not mock hardware, so nothing was sent. '
                  f'Replay runs on mock hardware only (ADR-13). CLI: {reason}')
    elif cause == 'start-state' and status == 'partial':
        detail = (f'{done} segment(s) were replayed, then the replay stopped because the simulated robot / positioner '
                  f'was not at the start of the next segment (that segment was not sent). CLI: {reason}')
    elif cause == 'start-state':
        detail = (f'The simulated robot / positioner is not at the start of segment {reason.split(":")[0]}; nothing was '
                  f'sent. Restart sim_mock (or bring it to the program start pose), then replay again. CLI: {reason}')
    else:
        detail = f'Replay {status or "error"}' + (f' after {done} segment(s)' if done else '') + f': {reason}'
    return {**base, 'type': f'/problems/replay-{cause}', 'cause': cause, 'replay_status': status, 'detail': detail}


def ros_domain() -> Optional[str]:
    return os.environ.get('ROS_DOMAIN_ID') or None


def replay_steps(settings: Settings, program: Path) -> List[Step]:
    launch = settings.replay_launch_line()
    return [Step('replay', settings.replay_argv() + ['--program', str(program)], result_prefix=RESULT_PREFIX,
                 explain=functools.partial(_explain_hook, launch=launch, domain=ros_domain()))]


def _explain_hook(rc: int, result: Optional[Dict[str, Any]], tail: List[str], launch: str,
                  domain: Optional[str]) -> Dict[str, Any]:
    return explain(rc, result, tail, launch, domain)


def info(settings: Settings, rev: Optional[str], state: Optional[str], problems: List[str]) -> Dict[str, Any]:
    """S4 block next to the button: the launch line to run, the domain and whether the program may be replayed."""
    return {'launch_command': settings.replay_launch_line(), 'ros_domain_id': ros_domain(),
            'program_rev': rev, 'program_state': state, 'problems': problems,
            'ready': rev is not None and state == 'ok',
            'note': 'Start sim_mock yourself with this line (same container and ROS domain as awm_api); the replay '
                    'sends joint motion to its mock controllers only. Weld events are not executed.'}
