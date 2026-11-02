"""SceneBackend on MoveIt2 ``move_group`` ``/check_state_validity`` (WP-S S.2).

States are converted from degrees to radians only through ``awm_core.units`` (ADR-14). One service call per state,
sequentially (batching is PLAN T1.8, only if TST-002 C5 > 30 s).
"""

from __future__ import annotations

from typing import List, Sequence

from awm_core import units
from awm_path.core.scene import StateCheck, validate_states
from awm_path.moveit_client import MoveItClient


class MoveGroupSceneBackend:
    """``SceneBackend`` backed by a running ``move_group`` (mock hardware / planning-only graph)."""

    name = 'move_group/check_state_validity'

    def __init__(self, client: MoveItClient) -> None:
        self._client = client

    def check(self, states_deg: Sequence[Sequence[float]]) -> List[StateCheck]:
        states = validate_states(states_deg, n_joints=len(self._client.joint_names))
        if states.shape[1] != 6:
            raise ValueError(f'MoveGroupSceneBackend expects 6 robot joints, client has {states.shape[1]}')
        out: List[StateCheck] = []
        for q_deg in states:
            valid, contacts = self._client.state_valid(units.deg_to_rad(q_deg))
            out.append(StateCheck(valid=bool(valid), contacts=tuple(sorted(contacts))))
        return out
