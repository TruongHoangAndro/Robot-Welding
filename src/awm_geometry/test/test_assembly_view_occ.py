"""CP-60: whole-file STEP assembly read per component (D20 solids with component paths, shells, stable ids).

Run with ``$AWM_OCC_PYTHON -m pytest`` (skipped without OCC; colcon runs it through test_occ_suite_in_conda.py).
"""

import json

import numpy as np


def test_solids_carry_component_paths(job_assembly):
    from awm_core.cad_roles import rel_paths

    _job, topo, tree = job_assembly
    assert topo['source']['components'] is True
    paths = {s['component_path'] for s in topo['solids']}
    assert {'Positioner:1 / Disc:1', 'Fixture:1 / Holder:1', 'Clamps:1 / Holder:1', 'Part:1 / PipeA:1',
            'Part:1 / PipeB:1', 'Robot:1'} <= paths
    rel = rel_paths(tree)
    for s in topo['solids']:
        assert rel[s['component_index']] == s['component_path']


def test_joint_samples_on_the_pipe_ring(job_assembly):
    """The joint id of export-view samples the ring of the two pipes from model.brep of `import` (same ids)."""
    from awm_geometry import joint_seam, testparts

    job, topo, _tree = job_assembly
    by_solid = {s['solid_id']: s['component_path'] for s in topo['solids']}
    ring = [j for j in topo['joints']
            if {by_solid[j['solid_a']], by_solid[j['solid_b']]} == {'Part:1 / PipeA:1', 'Part:1 / PipeB:1'}
            and abs(j['length_mm'] - 2 * np.pi * testparts.JOB_PIPE_R_MM) < 1e-3]
    assert len(ring) == 1
    doc = joint_seam.sample_joints(job, {'pitch_mm': 5.0, 'seams': [
        {'seam_id': 'S01', 'joint_edge_id': ring[0]['joint_edge_id'], 'reversed': False}]}, 'test')
    pts = np.array([p['p_mm'] for p in doc['seams'][0]['points']])
    c = np.array(testparts.JOB_AXIS_POINT_MM)
    np.testing.assert_allclose(pts[:, 0], testparts.JOB_JOINT_X_MM, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(pts[:, 1:] - c[1:], axis=1), testparts.JOB_PIPE_R_MM, atol=1e-6)


def test_shell_faces_exported_with_null_solid(tmp_path, occ):
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopExp import TopExp_Explorer

    from awm_geometry import cli, testparts

    shell = testparts.x_cylinder(10.0, 0.0, 20.0)
    faces = []
    exp = TopExp_Explorer(shell, TopAbs_FACE)
    while exp.More():
        faces.append(exp.Current())
        exp.Next()
    open_faces = testparts.separate(*faces[:1])          # the lateral face only: a surface model
    step = tmp_path / 'tool.step'
    testparts.write_tree_assembly(step, 'tool', [
        {'name': 'Mount', 'shape': testparts.box_mm((50, 0, 0), (60, 10, 10))},
        {'name': 'Body', 'shape': open_faces}])
    out = tmp_path / 'view'
    assert cli.main(['export-view', '--step', str(step), '--components', '--up-axis', '+Z', '--out', str(out)]) == 0
    topo = json.loads((out / 'topology.json').read_text())
    assert any(f['solid_id'] is None for f in topo['faces'])
    assert any(e['solid_id'] is None and e['curve_type'] == 'circle' for e in topo['edges'])


def test_untriangulated_face_is_a_listed_display_gap(job_assembly, tmp_path, monkeypatch):
    """A face BRepMesh cannot mesh (real CAD: small full-turn cones of the pulleys) is exported without a display
    mesh and listed in display_gaps in components mode; edges and ids are unchanged."""
    from awm_geometry import cli, view
    from awm_geometry.occ_io import GeometryError

    job, topo, _tree = job_assembly
    real = view.face_mesh
    calls = {'n': 0}

    def flaky(face):
        calls['n'] += 1
        if calls['n'] == 2:                                   # the second face never meshes
            raise GeometryError('face has no triangulation')
        return real(face)

    monkeypatch.setattr(view, 'face_mesh', flaky)
    monkeypatch.setattr(view, 'RETRY_ANGULAR_DEG', ())       # no retry succeeds either
    monkeypatch.setattr(view, '_rebuilt_mesh', lambda face: None)
    out = tmp_path / 'view'
    assert cli.main(['export-view', '--step', str(job.model_step), '--components', '--up-axis', '+Z',
                     '--out', str(out)]) == 0
    doc = json.loads((out / 'topology.json').read_text())
    assert doc['display_gaps'] == ['F2']
    assert [e['edge_id'] for e in doc['edges']] == [e['edge_id'] for e in topo['edges']]
