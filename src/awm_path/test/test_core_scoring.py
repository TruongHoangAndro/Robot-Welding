"""WP-S S.4 C5: scene scores of the S.3 candidates (F4 validity, F5 clearance levels, F9 contact) and the scene
columns of candidates.csv. Fake scene, synthetic plate; no ROS."""

import csv

import numpy as np
import pytest

from awm_path.core.contact import ContactParams, PartMesh
from awm_path.core.ladder import NodeScore
from awm_path.core.scene import StateCheck
from awm_path.core.scoring import SCENE_COLUMNS, SceneScoreParams, score_candidates
from awm_path.core.sweep import Candidate, write_candidates_csv


class FakeScene:
    """F4 invalid when q1 < 0; clearance level c passes when q2 >= c (q2 plays 'clearance in mm')."""

    name = 'fake'

    def __init__(self):
        self.calls = []

    def check(self, states, theta):
        self.calls.append(('check', len(states), theta))
        return [StateCheck(q[0] >= 0, () if q[0] >= 0 else ('a<->b',)) for q in states]

    def check_clearance(self, states, theta, c):
        self.calls.append(('clear', len(states), c))
        return [StateCheck(q[1] >= c, () if q[1] >= c else ('head_body<->part',)) for q in states]


def _plate():
    tris = []
    for i in range(60):
        for j in range(60):
            x, y = -30 + i, -30 + j
            tris += [[[x, y, 0], [x + 1, y, 0], [x + 1, y + 1, 0]], [[x, y, 0], [x + 1, y + 1, 0], [x, y + 1, 0]]]
    tris = np.asarray(tris, float)
    return PartMesh(tris, (tris[:, :, 0].mean(axis=1) > 0).astype(int), ('A', 'B'))


def _frame(x):
    """Seam frame on the plate: X = +Y part, Z = +Z part (out of the material), at (x, 0, 0)."""
    t = np.eye(4)
    t[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
    t[:3, 3] = [x, 0.0, 0.0]
    return t


def _cand(point, k, q, reason=None):
    return Candidate(point, k, 10.0 * k, 'S+E+W+' if q is not None else None,
                     tuple(q) if q is not None else None, None, reason)


def _layers():
    return [
        [_cand(0, 0, [1, 12, 0, 0, 0, 0]), _cand(0, 1, [1, 7, 0, 0, 0, 0]), _cand(0, 2, [-1, 20, 0, 0, 0, 0]),
         _cand(0, 3, [1, 3, 0, 0, 0, 0]), _cand(0, 4, None, 'no_ik_unreachable')],
        [_cand(1, 0, [1, 12, 0, 0, 0, 0])],          # point 1 is on face B only -> contact off seam (seam = A, A)
    ]


def test_scores_combine_validity_clearance_and_contact() -> None:
    scene = FakeScene()
    frames = [_frame(-10.0), _frame(15.0)]
    res = score_candidates(_layers(), frames, np.eye(4), 0.0, scene, _plate(), (('A',), ('A',)), ContactParams(),
                           SceneScoreParams(weld_clearance_mm=5.0, clearance_levels_mm=(5.0, 10.0)))
    s0 = res.scores[0]
    assert s0[0] == NodeScore(True, 10.0)            # F4 ok, >= 10 mm
    assert s0[1] == NodeScore(True, 5.0)             # F4 ok, 5 <= c < 10
    assert s0[2] == NodeScore(False, None)           # F4 invalid
    assert s0[3] == NodeScore(False, None)           # clearance < weld clearance
    assert s0[4] == NodeScore(False, None)           # kinematic reject: not checked
    assert res.scores[1] == [NodeScore(False, 10.0)]  # contact fails at point 1
    c = res.columns
    assert c['scene_valid'][0] == [True, True, False, True, None]
    assert c['clear_5mm'][0] == [True, True, None, False, None]
    assert c['clear_10mm'][0] == [True, False, None, None, None]
    assert c['contact_ok'][1] == [False] and c['contact_reason'][1] == ['contact_off_seam']
    assert c['node_valid'][0] == [True, True, False, False, False]
    assert c['scene_contacts'][0][2] == 'a<->b' and c['clearance_blocking'][0][3] == 'head_body<->part'
    assert set(c) == set(SCENE_COLUMNS)
    # one batched F4 call, then one clearance call per level on the survivors only
    assert scene.calls == [('check', 5, 0.0), ('clear', 4, 5.0), ('clear', 3, 10.0)]
    t = res.timing
    assert t['states_checked'] == 5 and t['points'] == 2 and t['contact_checks'] == 2
    for key in ('validity_s', 'clearance_s', 'contact_s', 'total_s', 'per_state_s'):
        assert t[key] >= 0.0


def test_weld_clearance_must_be_a_level() -> None:
    with pytest.raises(ValueError):
        SceneScoreParams(weld_clearance_mm=5.0, clearance_levels_mm=(10.0,))
    with pytest.raises(ValueError):
        SceneScoreParams(weld_clearance_mm=5.0, clearance_levels_mm=(10.0, 5.0))


def test_columns_fit_candidates_csv(tmp_path) -> None:
    layers = _layers()
    res = score_candidates(layers, [_frame(-10.0), _frame(15.0)], np.eye(4), 0.0, FakeScene(), _plate(),
                           (('A',), ('A',)), ContactParams(), SceneScoreParams())
    path = tmp_path / 'candidates.csv'
    write_candidates_csv(str(path), layers, res.columns)
    rows = list(csv.DictReader(path.open()))
    assert len(rows) == 6 and rows[0]['node_valid'] == 'true' and rows[4]['scene_valid'] == ''
