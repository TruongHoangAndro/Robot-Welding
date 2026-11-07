"""The planning description record (part of D07 ``planner.params``) must not depend on where the head folder lives:
the head content is covered by the head.yaml hash, the folder path is machine-specific (rule 5, no absolute paths).
Needs xacro and the ament share (runs inside awm_dev)."""
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'awm_description' / 'test'))

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_synthetic_head_dir  # noqa: E402


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
