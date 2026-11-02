"""SceneBackend: collision validity of robot joint states (WP-S S.2, B-06).

Public values are degrees (ADR-14). Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
This module never imports rclpy or ROS messages; ROS-bound implementations live in ``awm_path.backends``.
Clearance and closest pair are added by S.4.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol, Sequence, Tuple, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class StateCheck:
    """Result of one state check: ``valid`` and the sorted contact pairs ``'link_a<->link_b'``."""

    valid: bool
    contacts: Tuple[str, ...]


@runtime_checkable
class SceneBackend(Protocol):
    """Checks joint states (degrees) against the collision scene."""

    name: str

    def check(self, states_deg: Sequence[Sequence[float]]) -> List[StateCheck]:
        """Return one ``StateCheck`` per state, in the same order."""
        ...


def validate_states(states_deg: Sequence[Sequence[float]], n_joints: int = 6) -> np.ndarray:
    """Return the states as a float array of shape (N, n_joints) or raise ``ValueError``.

    A state with a different number of joints (e.g. robot + positioner = 7) is rejected, never truncated.
    """
    a = np.asarray(states_deg, dtype=float)
    if a.size == 0:
        return np.zeros((0, n_joints))
    if a.ndim != 2 or a.shape[1] != n_joints:
        raise ValueError(f'expected states of {n_joints} joints, got shape {a.shape}')
    if not np.all(np.isfinite(a)):
        raise ValueError('joint states contain non-finite values')
    return a
