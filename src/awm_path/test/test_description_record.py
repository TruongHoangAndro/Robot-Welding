"""The planning description record (part of D07 ``planner.params``) must not depend on where the head folder lives:
the head content is covered by the head.yaml hash, the folder path is machine-specific (rule 5, no absolute paths).
Needs xacro and the ament share (runs inside awm_dev)."""
import shutil

import yaml
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'awm_description' / 'test'))

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_map_override_head_dir, write_synthetic_head_dir  # noqa: E402


def test_record_is_independent_of_the_head_folder_location(tmp_path):
    from awm_seam import seam_axis
    from awm_path.description import load_planning_description

    job = make_ring_job(tmp_path / 'repo')
    frames = seam_axis.frames_path(job)
    head_a = write_synthetic_head_dir(tmp_path / 'a' / 'head')
    head_b = tmp_path / 'somewhere' / 'else' / 'head'
    shutil.copytree(head_a, head_b)

    rec_a = load_planning_description(frames, 'hcad', str(head_a)).as_record()
    rec_b = load_planning_description(frames, 'hcad', str(head_b)).as_record()

    assert rec_a == rec_b
    assert str(tmp_path) not in repr(rec_a)
    assert rec_a['xacro_args']['head_dir'] == '<head_dir>'


def test_acceleration_limits_come_from_the_program_sources(tmp_path):
    """G1.9: UR accelerations from joint_limits_v2.yaml (rad/s² → deg/s² via awm_core.units), the positioner from
    positioner_limits.yaml — the files the program generator reads; recorded with their hashes."""
    from awm_bringup.cell_config import load_positioner_limits, positioner_limits_path
    from awm_core import units
    from awm_core.hashing import sha256_file
    from awm_seam import seam_axis
    from awm_path.core.cell_urdf import UR_JOINTS
    from awm_path.description import joint_limits_file, load_planning_description

    job = make_ring_job(tmp_path / 'repo')
    desc = load_planning_description(seam_axis.frames_path(job), 'hbox')
    jl = yaml.safe_load(joint_limits_file().read_text())['joint_limits']
    want = [float(units.rad_to_deg(jl[j]['max_acceleration'])) for j in UR_JOINTS]
    want.append(load_positioner_limits(positioner_limits_path())['max_acceleration_deg_s2'])
    assert list(desc.sync_cell.acceleration_limits_deg_s2) == want
    rec = desc.as_record()
    assert rec['joint_limits_sha256'] == sha256_file(joint_limits_file())
    assert rec['acceleration_deg_s2'] == want


def test_rendered_descriptions_match_their_head_model(tmp_path):
    """Review I1: hbox renders a box head_body, hcad a mesh head_body (the label check of the runners)."""
    from awm_seam import seam_axis
    from awm_path.core.head import head_model_of_urdf
    from awm_path.description import load_planning_description
    job = make_ring_job(tmp_path / 'repo')
    frames = seam_axis.frames_path(job)
    head = write_synthetic_head_dir(tmp_path / 'h' / 'head')
    assert head_model_of_urdf(load_planning_description(frames, 'hbox').urdf) == 'hbox'
    assert head_model_of_urdf(load_planning_description(frames, 'hcad', str(head)).urdf) == 'hcad'


def test_conduit_source_of_the_rendered_head_is_in_the_record(tmp_path):
    """G1.13 request 2: the <awm_conduit source> of the description is in the head record (explicit cable exit,
    refused rear estimate, parametric H-BOX conduit); estimated sources are marked as Assumptions."""
    from awm_seam import seam_axis
    from awm_path.description import load_planning_description
    frames = seam_axis.frames_path(make_ring_job(tmp_path / 'repo'))
    explicit = write_synthetic_head_dir(tmp_path / 'e' / 'head')
    rear = write_map_override_head_dir(tmp_path / 'r' / 'head')
    cases = [(load_planning_description(frames, 'hcad', str(explicit)), 'explicit', False),
             (load_planning_description(frames, 'hcad', str(rear)), 'rear_estimate', True),
             (load_planning_description(frames, 'hbox'), 'parametric', True)]
    for desc, source, assumption in cases:
        rec = desc.as_record()['head']
        assert desc.head.conduit_source == source
        assert rec['conduit_source'] == source and rec['conduit_source_assumption'] is assumption


def test_hcad_with_nozzle_len_zero_loads_for_kinematics_and_refuses_f9(tmp_path):
    """G1.13 request 3: head.yaml nozzle_len_mm 0 -> empty nozzle links, tcp_laser at the head.yaml TCP; the
    kinematic description loads, F9 refuses (no disc, no fallback to the head mesh)."""
    import numpy as np
    from awm_seam import seam_axis
    from awm_path.core.head import NoNozzleError, contact_params_for
    from awm_path.description import load_planning_description
    frames = seam_axis.frames_path(make_ring_job(tmp_path / 'repo'))
    head_dir = write_synthetic_head_dir(tmp_path / 'z' / 'head', nozzle_len_mm=0.0)
    tcp = yaml.safe_load((head_dir / 'head.yaml').read_text())['tool0_T_tcp_mm']['xyz']
    desc = load_planning_description(frames, 'hcad', str(head_dir))
    assert desc.head.nozzle_geometry == 'none'
    assert np.allclose(desc.head.tool0_T_tcp_mm[:3, 3], tcp, atol=1e-6)
    assert desc.as_record()['head']['nozzle_radius_mm'] is None
    with pytest.raises(NoNozzleError):
        contact_params_for(desc.head)


def _strip_cable_exit(head_dir: Path) -> Path:
    """head.yaml without cable exit keys and without geometry_estimate -> the xacro uses the parametric conduit."""
    doc = yaml.safe_load((head_dir / 'head.yaml').read_text())
    doc.pop('cable_exit_tool0_mm', None)
    doc.pop('cable_axis_tool0', None)
    (doc.get('estimates') or {}).get('tcp', {}).pop('geometry_estimate', None)
    (head_dir / 'head.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    return head_dir


def test_hcad_without_cable_exit_records_the_parametric_conduit(tmp_path):
    """Review 2.d: H-CAD head.yaml with neither cable exit nor geometry_estimate -> conduit source parametric
    (Assumption) in the head record."""
    from awm_seam import seam_axis
    from awm_path.description import load_planning_description
    frames = seam_axis.frames_path(make_ring_job(tmp_path / 'repo'))
    head = _strip_cable_exit(write_synthetic_head_dir(tmp_path / 'p' / 'head'))
    desc = load_planning_description(frames, 'hcad', str(head))
    rec = desc.as_record()['head']
    assert desc.head_model == 'hcad' and desc.head.nozzle_geometry == 'cylinder'
    assert rec['conduit_source'] == 'parametric' and rec['conduit_source_assumption'] is True


@pytest.mark.parametrize('variant, source', [('parametric', 'parametric'), ('rear_estimate', 'rear_estimate')])
def test_no_nozzle_heads_with_estimated_conduit_sources(tmp_path, variant, source):
    """Review 2.d: nozzle_len_mm 0 with the parametric and the rear_estimate conduit sources -> nozzle none, source
    recorded as an Assumption, F9 refused."""
    from awm_seam import seam_axis
    from awm_path.core.head import NoNozzleError, contact_params_for
    from awm_path.description import load_planning_description
    frames = seam_axis.frames_path(make_ring_job(tmp_path / 'repo'))
    if variant == 'parametric':
        head = _strip_cable_exit(write_synthetic_head_dir(tmp_path / 'h' / 'head', nozzle_len_mm=0.0))
    else:
        head = write_map_override_head_dir(tmp_path / 'h' / 'head', nozzle_len_mm=0.0)
    desc = load_planning_description(frames, 'hcad', str(head))
    rec = desc.as_record()['head']
    assert rec['nozzle_geometry'] == 'none' and rec['nozzle_radius_mm'] is None
    assert rec['conduit_source'] == source and rec['conduit_source_assumption'] is True
    with pytest.raises(NoNozzleError):
        contact_params_for(desc.head)


@pytest.mark.parametrize('head_model', ['hbox', 'hcad'])
def test_urdf_hash_is_independent_of_the_install_prefix(tmp_path, head_model):
    """G1.17 follow-up: xacro writes the absolute path of its source file (install prefix of the checkout) into an XML
    comment; the record hash must not change with it, so a D07 planned in one checkout generates in another."""
    import re
    from awm_path.description import load_planning_description, location_free_urdf_sha256
    job = make_ring_job(tmp_path / 'repo')
    head = str(write_synthetic_head_dir(tmp_path / 'head')) if head_model == 'hcad' else None
    desc = load_planning_description(job.repo.cell_dir / 'model' / 'frames.yaml', head_model, head)
    prefixes = set(re.findall(r'(/\S*?)/install/awm_description/share', desc.urdf))
    assert prefixes, 'expected the xacro source path of awm_description in the rendered URDF'
    other = desc.urdf
    for prefix in prefixes:
        other = other.replace(prefix, '/elsewhere/other_checkout')
    assert other != desc.urdf
    assert location_free_urdf_sha256(other, head) == location_free_urdf_sha256(desc.urdf, head)
    assert location_free_urdf_sha256(desc.urdf + '<!-- note -->', head) == location_free_urdf_sha256(desc.urdf, head)
    # a real change outside comments still changes the hash
    changed = desc.urdf.replace('</robot>', '<link name="extra"/></robot>')
    assert location_free_urdf_sha256(changed, head) != location_free_urdf_sha256(desc.urdf, head)
