"""The real description render (``python3 -m awm_api.describe``, no fake): same URDF as the planner's
``load_planning_description`` (one source), mesh URIs rewritten, a package mesh served from the ament share.
Needs xacro and the installed awm_description / ur_description (runs inside awm_dev, ``colcon test``)."""

import shlex
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]
sys.path.insert(0, str(SRC / 'awm_path' / 'test'))
sys.path.insert(0, str(SRC / 'awm_description' / 'test'))

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

from conftest import auth  # noqa: E402
from awm_api.app import create_app  # noqa: E402
from awm_api.robot_model import API_MESHES, strip_comments  # noqa: E402
from awm_api.settings import Settings  # noqa: E402
from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_synthetic_head_dir  # noqa: E402


def test_real_render_matches_planner_description(tmp_path, users_file):
    from awm_path.description import load_planning_description

    root = tmp_path / 'repo'
    make_ring_job(root)
    head = write_synthetic_head_dir(root / 'cell' / 'model' / 'head')
    settings = Settings(repo_root=root, secrets_file=users_file,
                        describe_cmd=f'{shlex.quote(sys.executable)} -m awm_api.describe')
    with TestClient(create_app(settings)) as client:
        h = auth(client, 'op')
        r = client.get('/api/v1/cell/robot_description', headers=h)
        assert r.status_code == 200, r.text
        urdf = r.json()['urdf']
        planner = load_planning_description(root / 'cell' / 'model' / 'frames.yaml', 'hcad', str(head.resolve())).urdf
        back = urdf.replace(f'{API_MESHES}/head/', f'file://{head.resolve()}/') \
                   .replace(f'{API_MESHES}/package/', 'package://')
        assert back == strip_comments(planner.strip())
        assert '/install/' not in urdf and '<!--' not in urdf
        for link in ('tcp_laser', 'positioner_flange', 'head_body', 'wrist_3_link'):
            assert f'<link name="{link}"' in urdf
        assert str(tmp_path) not in urdf and 'file://' not in urdf
        assert f'{API_MESHES}/head/head_body.stl' in urdf
        pkg_mesh = f'{API_MESHES}/package/ur_description/meshes/ur5e/visual/base.dae'
        assert pkg_mesh in urdf
        r = client.get(pkg_mesh, headers=h)
        assert r.status_code == 200 and b'COLLADA' in r.content[:2000]
        assert client.get(f'{API_MESHES}/head/head_body.stl', headers=h).status_code == 200
