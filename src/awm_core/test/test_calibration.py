"""Active calibration loading and frame chain."""

import numpy as np
import pytest

from awm_core import frames, jsonio
from awm_core.calibration import load_active
from awm_core.repository import Repository


def test_load_active(tmp_path):
    repo = Repository(tmp_path)
    doc = jsonio.make_header('t', [], tmp_path)
    doc.update({'calibration_id': 'C1', 'status': 'NOMINAL',
                'base_T_fixture': frames.matrix_to_pose(frames.xyz_rpy_to_matrix([0.4, 0, 0.1],
                                                                                  [0, 0, 0.5])),
                'fixture_T_part': frames.matrix_to_pose(frames.xyz_rpy_to_matrix([0.01, 0, 0],
                                                                                  [0, 0, 0])),
                'tcp': {'tcp_mig': frames.matrix_to_pose(np.eye(4))}})
    jsonio.write_json(repo.active_calibration, doc, 'calibration')
    cal = load_active(repo)
    assert cal.calibration_id == 'C1' and len(cal.sha256) == 64
    p = cal.base_T_part @ np.array([0, 0, 0, 1.0])
    assert np.allclose(p[:3], [0.4 + 0.01 * np.cos(0.5), 0.01 * np.sin(0.5), 0.1])
    assert np.allclose(cal.tool0_T_tcp('tcp_mig'), np.eye(4))
    with pytest.raises(KeyError):
        cal.tool0_T_tcp('tcp_laser')
