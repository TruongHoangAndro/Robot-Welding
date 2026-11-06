"""CP-60 on the seam side: check 5 (seams only on joints between part solids) and the roles / axis inputs of
seam_axis.json (a role or axis edit makes the seam axis stale -> re-plan). Pure Python."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import jsonio  # noqa: E402
from awm_core.hashing import sha256_file  # noqa: E402
from awm_core.jsonio import StaleInputError  # noqa: E402
from awm_seam import joint_seams as js  # noqa: E402
from awm_seam import seam_axis as sa  # noqa: E402
from synthetic import fake_sampler, make_job, request  # noqa: E402


def with_roles(job, paths, assignments, axis=True):
    topo_path = job.dir / 'cache' / 'view' / 'topology.json'
    topo = json.loads(topo_path.read_text())
    topo['source']['components'] = True
    for s in topo['solids']:
        s['component_index'] = 1 if s['solid_id'] == 'B1' else 2
        s['component_path'] = paths[s['solid_id']]
    topo_path.write_text(json.dumps(topo))
    step = job.dir / 'model.step'
    roles = jsonio.make_header('test', [step], job.dir)
    roles.update({'schema_version': '2.0', 'scope': 'job', 'assignments': assignments,
                  'source': {'file': 'model.step', 'sha256': sha256_file(step), 'up_axis': '+Z'}})
    jsonio.write_json(job.dir / 'roles.json', roles, 'v2/cad_roles')
    if axis:
        pick = jsonio.make_header('test', [step], job.dir)
        pick.update({'schema_version': '2.0', 'source': {'file': 'model.step', 'sha256': sha256_file(step)},
                     'component_path': None, 'edge_id': 'E9', 'center_file_mm': [0, 0, 0], 'axis_file': [1, 0, 0],
                     'radius_mm': 60.0, 'reversed': False, 'method': 'pick'})
        jsonio.write_json(job.dir / 'axis_pick.json', pick, 'v2/axis_pick')
    return job


PARTS = {'B1': 'Part:1 / PipeA:1', 'B2': 'Part:1 / PipeB:1'}
PART_ROLES = [{'path': 'Part:1', 'role': 'part'}, {'path': 'Fixture:1', 'role': 'rotary_fixture'}]


def _build(job, **kw):
    return js.build_seams(job, [js.JointSeamRequest.from_dict(request(**kw))], 1.0, sampler=fake_sampler)


def test_edge_on_rotary_fixture_refused(tmp_path):
    job = with_roles(make_job(tmp_path), {'B1': 'Part:1 / PipeA:1', 'B2': 'Fixture:1 / Holder:1'}, PART_ROLES)
    with pytest.raises(js.SeamDefinitionError, match='E5 is on rotary_fixture component Fixture:1 / Holder:1'):
        _build(job, edges=['E5'])


def test_joint_part_to_holder_refused(tmp_path):
    job = with_roles(make_job(tmp_path), {'B1': 'Part:1 / PipeA:1', 'B2': 'Fixture:1 / Holder:1'}, PART_ROLES)
    with pytest.raises(js.SeamDefinitionError, match='only joints between two part solids'):
        _build(job, edges=['E1'])


def test_part_joint_accepted(tmp_path):
    job = with_roles(make_job(tmp_path), PARTS, PART_ROLES)
    doc = _build(job, edges=['E1'])
    assert [s['seam_id'] for s in doc['seams']] == ['S01']


def test_roles_edit_makes_seam_axis_stale(tmp_path):
    job = with_roles(make_job(tmp_path), PARTS, PART_ROLES)
    _build(job, edges=['E1'])
    doc = sa.build_seam_axis(job)
    names = {i['name'] for i in doc['inputs']}
    assert {'model.step', 'roles.json', 'axis_pick.json', 'placement.json', 'seams.json'} <= names
    sa.read_seam_axis(job)                                     # fresh
    roles = json.loads((job.dir / 'roles.json').read_text())
    roles['assignments'].append({'path': 'Clamps:1', 'role': 'exclude'})
    (job.dir / 'roles.json').write_text(json.dumps(roles))
    with pytest.raises(StaleInputError):
        sa.read_seam_axis(job)


def test_jobs_without_roles_unchanged(tmp_path):
    job = make_job(tmp_path)
    _build(job, edges=['E1'])
    names = {i['name'] for i in sa.build_seam_axis(job)['inputs']}
    assert names == {'seams.json', 'placement.json', '../../cell/model/frames.yaml'}
