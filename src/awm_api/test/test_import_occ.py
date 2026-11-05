"""End-to-end upload with the real OCC process (ADR-15): synthetic STEP from awm_geometry make-test-part."""

import json
import os

import pytest
from fastapi.testclient import TestClient

from awm_api.app import create_app
from awm_core import occ_bridge
from conftest import auth, wait_run


@pytest.fixture
def occ_client(settings):
    if not os.environ.get(occ_bridge.ENV_VAR):
        pytest.skip(f'{occ_bridge.ENV_VAR} not set (the OCC conda interpreter, ADR-15)')
    settings.occ_python_cmd = None  # real interpreter from the environment
    with TestClient(create_app(settings)) as c:
        yield c


def test_upload_synthetic_step_end_to_end(occ_client, repo, tmp_path):
    step = tmp_path / 'tube_tube.step'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'tube_tube', '--out', str(step)])
    h = auth(occ_client, 'eng')
    with step.open('rb') as fh:
        r = occ_client.post('/api/v1/jobs', headers=h, data={'part_no': 'TT', 'part_rev': 'A'},
                            files={'step': ('tube_tube.step', fh, 'application/octet-stream')})
    assert r.status_code == 202, r.text
    run = wait_run(occ_client, h, r.json()['run_id'], timeout_s=180)
    assert run['state'] == 'succeeded', run
    topo = occ_client.get('/api/v1/jobs/TT_A/view/topology.json', headers=h).json()
    assert topo['schema_version'] == '2.0' and topo['unit'] == 'mm'
    assert topo['source']['file'] == 'model.step'
    assert len(topo['joints']) == 2
    glb = occ_client.get('/api/v1/jobs/TT_A/view/view.glb', headers=h)
    assert glb.status_code == 200 and glb.content[:4] == b'glTF'
    detail = occ_client.get('/api/v1/jobs/TT_A', headers=h).json()
    assert detail['status'] == 'draft' and detail['part_no'] == 'TT' and detail['placement'] is None
    part = json.loads((repo / 'jobs/TT_A/part.json').read_text())
    assert part['step_file'] == 'tube_tube.step'
