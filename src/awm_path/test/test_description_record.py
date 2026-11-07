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
