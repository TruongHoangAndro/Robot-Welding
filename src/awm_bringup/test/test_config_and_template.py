"""D16 configuration and repository template: valid, and consistent with the cell model (D14)."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from awm_core import jsonio
from awm_core.frames import quat_to_matrix, rpy_to_matrix
from awm_core.repository import Repository

PKG = Path(__file__).resolve().parents[1]
TEMPLATE = PKG / 'repo_template'


def _description_config(name):
    from ament_index_python.packages import get_package_share_directory
    return yaml.safe_load((Path(get_package_share_directory('awm_description')) / 'config' /
                           name).read_text())


def _params():
    return yaml.safe_load((PKG / 'config' / 'awm_params.yaml').read_text())['/**']['ros__parameters']


def test_params_file():
    p = _params()
    assert p['repo_root'] == ''  # no absolute path in source
    assert len(p['home_joints_rad']) == 6 and len(p['joint_names']) == 6
    assert 0 < p['velocity_scaling'] <= 0.2 and 0 < p['acceleration_scaling'] <= 0.2
    assert p['controller_action'].endswith('follow_joint_trajectory')


def test_home_matches_mock_start_state():
    initial = _description_config('initial_positions.yaml')
    p = _params()
    assert [initial[j] for j in p['joint_names']] == pytest.approx(p['home_joints_rad'])


def test_template_files_validate():
    for wps in sorted((TEMPLATE / 'cell' / 'wps').glob('*.json')):
        doc = jsonio.read_json(wps, 'wps')
        assert doc['status'] == 'SAMPLE' and wps.stem == doc['wps_id']
    for cal in (TEMPLATE / 'cell' / 'calibration').glob('*.json'):
        assert jsonio.read_json(cal, 'calibration')['status'] == 'NOMINAL'


def test_nominal_calibration_matches_cell_model():
    cell = _description_config('cell_nominal.yaml')
    cal = jsonio.read_json(TEMPLATE / 'cell' / 'calibration' / 'active.json', 'calibration')
    assert np.allclose(cal['base_T_fixture']['position_m'], cell['fixture']['xyz_m'])
    assert np.allclose(quat_to_matrix(cal['base_T_fixture']['quat_xyzw']),
                       rpy_to_matrix(*cell['fixture']['rpy_rad']))
    tcp = cal['tcp']['tcp_mig']
    assert np.allclose(tcp['position_m'], cell['torch_mig']['tcp_xyz_m'])
    assert np.allclose(quat_to_matrix(tcp['quat_xyzw']),
                       rpy_to_matrix(*cell['torch_mig']['tcp_rpy_rad']), atol=1e-9)


def test_repository_init_from_template(tmp_path):
    repo = Repository(tmp_path / 'repo')
    repo.init_from_template(TEMPLATE)
    assert repo.active_calibration.is_file()
    assert len(list(repo.wps_dir.glob('*.json'))) == 3
