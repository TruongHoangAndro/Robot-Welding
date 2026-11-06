"""Vectorised UR5e kinematics for the 7-axis sweep (G1.3; ADR-12; TST-002 Rev 0.3 draft §5.1 F2, F6).

* :func:`batch_fk` — ``base_link_T_tool0`` (mm) for many joint states, standard UR DH of ``states.UR5E_GEOMETRY``
  (Fact: equal to ``ur_description`` default kinematics; checked against ``UrAnalyticKinematics.fk`` in the tests);
* :func:`batch_label_metrics` — configuration label index (``states.LABELS`` order) and the three F6 metrics,
  identical to ``states.label_and_metrics``;
* :func:`ik_by_label` — all analytic IK solutions of many ``tool0`` poses, one per configuration label, each
  verified by FK (same tolerances as ``UrAnalyticKinematics.ik``).

The IK itself is the ``KinematicsBackend`` library (``ur-analytic-ik``); only FK verification and labelling are
vectorised. Public values mm / degrees, radians only through ``awm_core.units`` (ADR-14). Pure numpy, no rclpy (B-06).
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from awm_core import units
from awm_path.core.kinematics import LIB_BASE_T_BASE_LINK, wrap_deg
from awm_path.core.states import UR5E_GEOMETRY, UrGeometry

#: Number of configuration labels (S±E±W±), index = 4·[S−] + 2·[E−] + [W−] = ``states.LABELS`` order.
LABEL_COUNT = 8
_ALPHA_RAD = units.deg_to_rad(np.array([90.0, 0.0, 0.0, 90.0, -90.0, 0.0]))
_SIN_ZERO = 1e-12
_MM_ZERO = 1e-9
_IK_CHECK_POS_MM = 1e-3
_IK_CHECK_ROT_RAD = 1e-7
_BASE_LINK_T_LIB = np.linalg.inv(LIB_BASE_T_BASE_LINK)


def _dh_params(geo: UrGeometry) -> Tuple[np.ndarray, np.ndarray]:
    return (np.array([geo.d1, 0.0, 0.0, geo.d4, geo.d5, geo.d6]),
            np.array([0.0, geo.a2, geo.a3, 0.0, 0.0, 0.0]))


def _chain(q_rad: np.ndarray, n_joints: int, geo: UrGeometry) -> np.ndarray:
    """``lib_T_frame_n`` (N, 4, 4) in mm after the first ``n_joints`` DH transforms."""
    d, a = _dh_params(geo)
    n = q_rad.shape[0]
    eye = np.eye(4)
    c0, c1, c2, c3 = (np.broadcast_to(eye[:, k], (n, 4)).copy() for k in range(4))   # columns of t, (n, 4)
    for i in range(n_joints):
        ct, st = np.cos(q_rad[:, i])[:, None], np.sin(q_rad[:, i])[:, None]
        ca, sa = float(np.cos(_ALPHA_RAD[i])), float(np.sin(_ALPHA_RAD[i]))
        # t @ DH(theta, d, a, alpha), column by column (the DH matrix is sparse)
        x = c0 * ct + c1 * st
        y = c1 * ct - c0 * st
        c3 = x * a[i] + c2 * d[i] + c3
        c0, c1, c2 = x, y * ca + c2 * sa, c2 * ca - y * sa
    return np.stack([c0, c1, c2, c3], axis=2)


def _as_q(q_deg: np.ndarray) -> np.ndarray:
    q = np.asarray(q_deg, dtype=float)
    if q.ndim != 2 or q.shape[1] != 6:
        raise ValueError(f'expected joint states of shape (N, 6), got {q.shape}')
    return units.deg_to_rad(q)


def batch_fk(q_deg: np.ndarray, geo: UrGeometry = UR5E_GEOMETRY) -> np.ndarray:
    """``base_link_T_tool0`` (N, 4, 4), translation in mm, for joint states (N, 6) in degrees."""
    return _BASE_LINK_T_LIB @ _chain(_as_q(q_deg), 6, geo)


def batch_label_metrics(q_deg: np.ndarray, geo: UrGeometry = UR5E_GEOMETRY
                        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(label index, |sin q5|, |sin q3|, |u| mm) for joint states (N, 6) in degrees — ``states.label_and_metrics``."""
    q = _as_q(q_deg)
    wc = _chain(q, 5, geo)[:, :3, 3]
    u = np.cos(q[:, 0]) * wc[:, 0] + np.sin(q[:, 0]) * wc[:, 1]
    s3, s5 = np.sin(q[:, 2]), np.sin(q[:, 4])
    lab = 4 * (u < -_MM_ZERO) + 2 * (s3 < -_SIN_ZERO) + (s5 < -_SIN_ZERO)
    return lab.astype(np.int64), np.abs(s5), np.abs(s3), np.abs(u)


def _rotation_error(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    r = np.swapaxes(a[:, :3, :3], 1, 2) @ b[:, :3, :3]
    s = 0.5 * np.linalg.norm(np.stack([r[:, 2, 1] - r[:, 1, 2], r[:, 0, 2] - r[:, 2, 0], r[:, 1, 0] - r[:, 0, 1]], 1),
                             axis=1)
    c = 0.5 * (np.trace(r, axis1=1, axis2=2) - 1.0)
    return np.arctan2(s, c)


def ik_by_label(kin, base_link_T_tool0: np.ndarray, with_metrics: bool = False):
    """All IK solutions of N ``tool0`` poses (N, 4, 4; mm), indexed by configuration label.

    Returns ``(q, ok)``: ``q`` (N, 8, 6) degrees in (−180, 180] (NaN where no solution), ``ok`` (N, 8). A solution is
    kept only if its FK reproduces the target within 1e-3 mm / 1e-7 rad (filters degenerate solutions, as
    ``UrAnalyticKinematics.ik``); if two solutions share a label the first in library order is kept.
    ``kin`` must expose the library (``UrAnalyticKinematics``). ``with_metrics`` also returns the F6 metrics
    (|sin q5|, |sin q3|, |u| mm), each (N, 8), NaN where no solution."""
    targets = np.asarray(base_link_T_tool0, dtype=float).reshape(-1, 4, 4)
    n = len(targets)
    q_out = np.full((n, LABEL_COUNT, 6), np.nan)
    ok = np.zeros((n, LABEL_COUNT), dtype=bool)
    mets = [np.full((n, LABEL_COUNT), np.nan) for _ in range(3)]

    def _out():
        return (q_out, ok, *mets) if with_metrics else (q_out, ok)

    lib = kin._lib  # noqa: SLF001 — the backend's own library, no second IK source
    lib_targets = LIB_BASE_T_BASE_LINK @ targets
    lib_targets[:, :3, 3] = units.mm_to_m(lib_targets[:, :3, 3])
    lib_targets = np.ascontiguousarray(lib_targets)
    raw = [lib.inverse_kinematics(lib_targets[k]) for k in range(n)]
    counts = np.array([len(r) for r in raw], dtype=np.int64)
    if not counts.sum():
        return _out()
    sols_rad = np.array([x for r in raw for x in r], dtype=float).reshape(-1, 6)
    rows = np.repeat(np.arange(n), counts)
    fin = np.all(np.isfinite(sols_rad), axis=1)
    rows, sols_rad = rows[fin], sols_rad[fin]
    if not len(rows):
        return _out()
    rows_a = rows
    q_deg = wrap_deg(units.rad_to_deg(sols_rad))
    fk = batch_fk(q_deg)
    dp = np.linalg.norm(fk[:, :3, 3] - targets[rows_a, :3, 3], axis=1)
    dr = _rotation_error(fk, targets[rows_a])
    good = (dp < _IK_CHECK_POS_MM) & (dr < _IK_CHECK_ROT_RAD)
    lab, s5, s3, u = batch_label_metrics(q_deg)
    idx = np.nonzero(good)[0]
    _, first = np.unique(rows_a[idx] * LABEL_COUNT + lab[idx], return_index=True)   # first in library order
    idx = idx[first]
    q_out[rows_a[idx], lab[idx]] = q_deg[idx]
    ok[rows_a[idx], lab[idx]] = True
    for m, v in zip(mets, (s5, s3, u)):
        m[rows_a[idx], lab[idx]] = v[idx]
    return _out()


__all__ = ['LABEL_COUNT', 'batch_fk', 'batch_label_metrics', 'ik_by_label']
