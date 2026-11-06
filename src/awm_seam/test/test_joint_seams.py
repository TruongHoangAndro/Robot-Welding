"""G1.2 / T2.2: picked edge ids -> joint edge -> D04 v2 seams.json (pure Python, fake sampler)."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import jsonio  # noqa: E402
from awm_seam import joint_seams as js  # noqa: E402
from synthetic import fake_sampler, make_job, request, topology_doc  # noqa: E402


@pytest.fixture
def job(tmp_path):
    return make_job(tmp_path)


def test_either_edge_of_the_joint_gives_the_same_joint(job):
    topo = js.Topology.load(job)
    assert topo.resolve_joint(['E1'])['joint_edge_id'] == 'JE1'
    assert topo.resolve_joint(['E5'])['joint_edge_id'] == 'JE1'
    assert topo.resolve_joint(['E5', 'E1'])['joint_edge_id'] == 'JE1'


@pytest.mark.parametrize('edges, text', [(['E2'], 'not a joint'), (['E99'], 'unknown edge'),
                                         ([], 'no edge')])
def test_bad_picks_are_refused(job, edges, text):
    with pytest.raises(js.SeamDefinitionError, match=text):
        js.Topology.load(job).resolve_joint(edges)


def test_picks_of_two_joints_are_refused(tmp_path):
    (tmp_path / 'model.step').write_text('x')
    doc = topology_doc(tmp_path)
    doc['edges'][1]['joint_edge_id'] = 'JE2'
    doc['joints'].append(dict(doc['joints'][0], joint_edge_id='JE2', edge_a='E2', edge_b='E2'))
    topo = js.Topology(doc)
    with pytest.raises(js.SeamDefinitionError, match='two joints'):
        topo.resolve_joint(['E1', 'E2'])


def test_build_writes_valid_seams_v2(job):
    doc = js.build_seams(job, [js.JointSeamRequest.from_dict(
        request(arcs=2, arc_overlap_mm=3.0, positioner_mode='B', positioner_angles_deg=[0.0, 180.0]))],
        pitch_mm=1.0, sampler=fake_sampler)
    jsonio.validate(doc, 'v2/seams')
    on_disk = jsonio.read_json(job.seams_json, 'v2/seams', base_dir=job.dir)
    seam = on_disk['seams'][0]
    assert on_disk['schema_version'] == js.SCHEMA_VERSION
    assert seam['joint_edge_id'] == 'JE1' and seam['solids'] == ['B1', 'B2']
    assert seam['edges'] == ['E1', 'E5']
    assert seam['faces'] == {'n1': ['F1'], 'n2': ['F4']}
    assert seam['positioner_mode'] == 'B' and seam['positioner_angles_deg'] == [0.0, 180.0]
    assert seam['geometry']['spacing_mm'] <= 1.0
    steps = [b['s_mm'] - a['s_mm'] for a, b in zip(seam['geometry']['points'], seam['geometry']['points'][1:])]
    assert max(steps) <= 1.0 + 1e-9
    assert {i['name'] for i in on_disk['inputs']} == {'part.json', 'cache/view/topology.json'}


def test_default_mode_is_sync_without_angles(job):
    doc = js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=fake_sampler)
    seam = doc['seams'][0]
    assert seam['positioner_mode'] == 'sync' and seam['positioner_angles_deg'] is None
    assert seam['arcs'] == 1


@pytest.mark.parametrize('kw, text', [
    ({'positioner_mode': 'B'}, 'one angle per arc'),
    ({'positioner_mode': 'B', 'arcs': 2, 'positioner_angles_deg': [10.0]}, 'one angle per arc'),
    ({'positioner_mode': 'C', 'positioner_angles_deg': [10.0]}, 'only for mode B'),
    ({'positioner_mode': 'robot_only'}, 'positioner_mode'),
    ({'arcs': 0}, 'arcs'),
    ({'arcs': 2, 'arc_overlap_mm': 50.0}, 'overlap'),
    ({'seam_id': 'J1'}, 'seam id'),
    ({'joint_type': 'weave'}, 'joint type'),
    ({'start_offset_mm': -1.0}, 'start'),
    ({'bogus': 1}, 'unknown'),
])
def test_attribute_errors(job, kw, text):
    with pytest.raises(js.SeamDefinitionError, match=text):
        js.build_seams(job, [js.JointSeamRequest.from_dict(request(**kw))], 1.0, sampler=fake_sampler)


def test_pitch_ceiling(job):
    with pytest.raises(js.SeamDefinitionError, match='pitch'):
        js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.5, sampler=fake_sampler)


def test_one_joint_in_two_seams_and_duplicate_ids_are_refused(job):
    a = js.JointSeamRequest.from_dict(request())
    b = js.JointSeamRequest.from_dict(request(seam_id='S02', sequence=2, edges=['E5']))
    with pytest.raises(js.SeamDefinitionError, match='JE1'):
        js.build_seams(job, [a, b], 1.0, sampler=fake_sampler)
    c = js.JointSeamRequest.from_dict(request(sequence=2))
    with pytest.raises(js.SeamDefinitionError, match='duplicate'):
        js.build_seams(job, [a, c], 1.0, sampler=fake_sampler)


def test_sampler_output_for_another_joint_is_refused(job):
    def wrong(job_, req):
        out = fake_sampler(job_, req)
        out['seams'][0]['joint_edge_id'] = 'JE7'
        return out
    with pytest.raises(js.SeamDefinitionError, match='sampler'):
        js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=wrong)


def test_stale_topology_refused(job):
    js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=fake_sampler)
    topo = job.dir / 'cache' / 'view' / 'topology.json'
    doc = json.loads(topo.read_text())
    doc['created_by'] = 'changed'
    topo.write_text(json.dumps(doc))
    with pytest.raises(jsonio.StaleInputError):
        js.load_seams(job)


def test_sampler_request_carries_direction_and_start(job):
    seen = {}

    def spy(job_, req):
        seen.update(req)
        return fake_sampler(job_, req)
    js.build_seams(job, [js.JointSeamRequest.from_dict(request(reversed=True, start_offset_mm=5.0))], 0.5,
                   sampler=spy)
    assert seen['pitch_mm'] == 0.5
    assert seen['seams'][0] == {'seam_id': 'S01', 'joint_edge_id': 'JE1', 'reversed': True,
                                'start_offset_mm': 5.0, 'start_point_mm': None}


def test_schema_mode_b_needs_angles_and_others_refuse_them():
    ex = json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'seams.example.json').read_text())
    jsonio.validate(ex, 'v2/seams')
    seam = ex['seams'][0]
    seam.update(positioner_mode='B', positioner_angles_deg=None)
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(ex, 'v2/seams')
    seam.update(positioner_angles_deg=[0.0])
    jsonio.validate(ex, 'v2/seams')
    seam.update(positioner_mode='sync')
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(ex, 'v2/seams')
    seam.update(positioner_mode='robot_only', positioner_angles_deg=None)
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(ex, 'v2/seams')


def test_seam_axis_example_is_consistent_with_the_seams_example():
    from awm_seam import seam_axis as sa
    v2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'
    ax = json.loads((v2 / 'seam_axis.example.json').read_text())
    seams = json.loads((v2 / 'seams.example.json').read_text())
    jsonio.validate(ax, 'v2/seam_axis')
    for a, s in zip(ax['seams'][0]['points'], seams['seams'][0]['geometry']['points']):
        back = sa.axis_to_part(ax, a)
        assert max(abs(x - y) for x, y in zip(back['p_mm'], s['p_mm'])) < 1e-6
