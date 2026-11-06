"""CP-60 cross-check math (pure numpy, no OCC): job STEP vs cell model.

Check 1 (positioner match): the job file has its own origin, so only quantities relative to the picked axis are
compared — the picked circle radius and, for every ``rotary_fixture`` component present once in both files (instance
name), its pose in the axis frame. The job positioner may stand at another angle than in the cell file: the angle φ
about the axis is taken from the first match (cell = Rx(φ) · job) and every match must agree with it.
Checks 2 / 3: pose offsets (robot base, tool) against the cell. Check 4: the rotating set swept over a full turn vs
the static objects, per 1 mm slab along the axis and per rotating component an interval [r_min, r_max].
Lengths mm, angles deg (ADR-14).
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List

import numpy as np

from awm_core import frames, units

ORDER = {'green': 0, 'yellow': 1, 'red': 2}


def worst(statuses: Iterable[str]) -> str:
    """Worst of green / yellow / red; ``skipped`` does not count (all skipped -> green)."""
    found = [s for s in statuses if s in ORDER]
    return max(found, key=ORDER.get) if found else 'green'


def _rx(deg: float) -> np.ndarray:
    return frames.make_transform([0.0, 0.0, 0.0], frames.axis_angle_to_matrix([1, 0, 0], units.deg_to_rad(deg)))


def _offset(t: np.ndarray) -> tuple:
    """(translation mm, rotation deg) of a 4x4 against identity."""
    return float(np.linalg.norm(t[:3, 3])), math.degrees(frames.rotation_angle_between(t[:3, :3], np.eye(3)))


def positioner_match(cell: Dict[str, Any], job: Dict[str, Any], tol_mm: float, tol_deg: float) -> Dict[str, Any]:
    """Check 1. ``cell`` / ``job``: ``{radius_mm, axis_T: {instance name: 4x4 axis_T_component}, duplicates: [...]}``.

    Returns ``{status, values, tolerance, note, phi_deg}`` (status green / red; phi None when not determined).
    """
    tolerance = {'mm': tol_mm, 'deg': tol_deg}
    notes: List[str] = []
    matched = sorted(set(cell['axis_T']) & set(job['axis_T']))
    dup = sorted(set(cell.get('duplicates') or []) | set(job.get('duplicates') or []))
    d_radius = abs(float(job['radius_mm']) - float(cell['radius_mm']))
    values: Dict[str, Any] = {'radius_job_mm': float(job['radius_mm']), 'radius_cell_mm': float(cell['radius_mm']),
                              'radius_diff_mm': d_radius, 'matched': matched}
    if d_radius > tol_mm:
        notes.append(f'picked circle radius {job["radius_mm"]:.3f} mm differs from the cell pick '
                     f'{cell["radius_mm"]:.3f} mm by {d_radius:.3f} mm (another circle?)')
    if dup:
        notes.append('rotary_fixture instance name(s) not unique: ' + ', '.join(dup))
    if not matched:
        notes.append('positioner cannot be confirmed: no rotary_fixture component of the job file has the same '
                     'instance name as one of the cell')
        return {'status': 'red', 'values': values, 'tolerance': tolerance, 'note': '; '.join(notes), 'phi_deg': None}
    delta0 = np.asarray(cell['axis_T'][matched[0]]) @ frames.invert(np.asarray(job['axis_T'][matched[0]]))
    phi = math.degrees(math.atan2(delta0[2, 1], delta0[1, 1]))
    worst_mm = worst_deg = 0.0
    for name in matched:
        delta = np.asarray(cell['axis_T'][name]) @ frames.invert(np.asarray(job['axis_T'][name]))
        mm, deg = _offset(_rx(-phi) @ delta)
        worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)
        if mm > tol_mm or deg > tol_deg:
            notes.append(f'{name}: {mm:.3f} mm / {deg:.3f} deg from the cell after the turn of {phi:.3f} deg')
    values.update(max_offset_mm=worst_mm, max_angle_deg=worst_deg)
    status = 'red' if notes else 'green'
    return {'status': status, 'values': values, 'tolerance': tolerance,
            'note': '; '.join(notes) if notes else f'{len(matched)} component(s) agree; phi {phi:.3f} deg',
            'phi_deg': phi}


def pose_check(cell_T: np.ndarray, job_T: np.ndarray, tol_mm: float, tol_deg: float) -> Dict[str, Any]:
    """Checks 2 / 3: green within tolerance, else yellow (the cell value is used)."""
    mm, deg = _offset(frames.invert(np.asarray(cell_T)) @ np.asarray(job_T))
    ok = mm <= tol_mm and deg <= tol_deg
    return {'status': 'green' if ok else 'yellow', 'values': {'offset_mm': mm, 'angle_deg': deg},
            'tolerance': {'mm': tol_mm, 'deg': tol_deg}}


def _intervals(points: np.ndarray, slab_mm: float, lo: int, hi: int) -> tuple:
    """Per slab k in [lo, hi] (merged with its neighbours): (r_min, r_max); empty slabs give (+inf, -inf)."""
    k = np.floor(points[:, 0] / slab_mm).astype(int)
    r = np.hypot(points[:, 1], points[:, 2])
    n = hi - lo + 1
    rmin, rmax = np.full(n + 2, np.inf), np.full(n + 2, -np.inf)
    inside = (k >= lo) & (k <= hi)
    np.minimum.at(rmin, k[inside] - lo + 1, r[inside])
    np.maximum.at(rmax, k[inside] - lo + 1, r[inside])
    # merge neighbours (conservative by one slab against sparse samples at the slab borders)
    mn = np.minimum(np.minimum(rmin[:-2], rmin[1:-1]), rmin[2:])
    mx = np.maximum(np.maximum(rmax[:-2], rmax[1:-1]), rmax[2:])
    return mn, mx


def swept_envelope_hits(rotating: Dict[str, np.ndarray], statics: Dict[str, np.ndarray],
                        slab_mm: float = 1.0) -> List[Dict[str, Any]]:
    """Check 4: static samples inside the swept volume of a rotating component (both in the axis frame, x = axis).

    One record per (static, rotating) pair with hits: ``{static, rotating, count, depth_mm, at_x_mm}``
    (depth = deepest sample's distance to the interval border).
    """
    hits: List[Dict[str, Any]] = []
    for rname, rpts in rotating.items():
        rpts = np.asarray(rpts, dtype=float)
        if not len(rpts):
            continue
        lo = int(np.floor(rpts[:, 0].min() / slab_mm)) - 1
        hi = int(np.floor(rpts[:, 0].max() / slab_mm)) + 1
        mn, mx = _intervals(rpts, slab_mm, lo, hi)
        for sname, spts in statics.items():
            spts = np.asarray(spts, dtype=float)
            if not len(spts):
                continue
            k = np.floor(spts[:, 0] / slab_mm).astype(int)
            sel = (k >= lo) & (k <= hi)
            if not sel.any():
                continue
            r = np.hypot(spts[sel, 1], spts[sel, 2])
            a, b = mn[k[sel] - lo], mx[k[sel] - lo]
            inside = (r > a) & (r < b)
            if inside.any():
                depth = np.minimum(r - a, b - r)[inside]
                j = int(np.argmax(depth))
                hits.append({'static': sname, 'rotating': rname, 'count': int(inside.sum()),
                             'depth_mm': float(depth[j]), 'at_x_mm': float(spts[sel][inside][j, 0])})
    return hits


def triangles_near_cylinder(verts: np.ndarray, tris: np.ndarray, x_lo: float, x_hi: float, r_max: float,
                            margin: float = 1.0) -> np.ndarray:
    """Indices of the triangles whose bounding box overlaps the box around the swept cylinder (axis frame:
    x in [x_lo, x_hi], |y|, |z| <= r_max, grown by ``margin``). Conservative prefilter of check 4: only these
    triangles of a static object are sampled (a large frame far from the positioner costs nothing)."""
    v = np.asarray(verts, dtype=float)[np.asarray(tris, dtype=int)]  # (n, 3, 3)
    lo, hi = v.min(axis=1), v.max(axis=1)
    r = r_max + margin
    ok = (hi[:, 0] >= x_lo - margin) & (lo[:, 0] <= x_hi + margin)
    for k in (1, 2):
        ok &= (hi[:, k] >= -r) & (lo[:, k] <= r)
    return np.nonzero(ok)[0]
