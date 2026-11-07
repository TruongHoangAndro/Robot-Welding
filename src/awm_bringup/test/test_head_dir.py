"""check_head_dir (G1.1): the launch refuses an H-CAD head folder with missing or changed files, before xacro runs."""

import sys
from pathlib import Path

import pytest
import yaml

from awm_bringup.cell_config import HeadDirError, check_head_dir

# Synthetic head folder writer shared with awm_description (test data only, same repository).
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_description' / 'test'))
from head_fixture import real_head_dir, write_synthetic_head_dir  # noqa: E402


# The real export (AWM_HEAD_DIR) is checked when it is named; no skip when it is not.
@pytest.mark.parametrize('kind', ['synthetic'] + (['real'] if real_head_dir() else []))
def test_valid(kind, tmp_path):
    folder = write_synthetic_head_dir(tmp_path / 'h') if kind == 'synthetic' else real_head_dir()
    files = check_head_dir(str(folder))
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    expected = {'head.yaml'} | {doc['links'][n][k]['file'] for n in ('head_body', 'head_mount')
                                for k in ('mesh', 'hull')}
    assert sorted(p.name for p in files) == sorted(expected)


@pytest.mark.parametrize('value', ['', '   ', 'cell/model/head'])
def test_empty_or_relative(value):
    with pytest.raises(HeadDirError, match='absolute'):
        check_head_dir(value)


def test_missing_head_yaml(tmp_path):
    with pytest.raises(HeadDirError, match='head.yaml'):
        check_head_dir(str(tmp_path))


def test_missing_mesh(tmp_path):
    folder = write_synthetic_head_dir(tmp_path / 'h')
    (folder / 'head_body_hull.stl').unlink()
    with pytest.raises(HeadDirError, match='head_body_hull.stl'):
        check_head_dir(str(folder))


def test_changed_mesh(tmp_path):
    folder = write_synthetic_head_dir(tmp_path / 'h')
    with open(folder / 'head_mount.stl', 'ab') as f:
        f.write(b'x')
    with pytest.raises(HeadDirError, match='SHA-256.*head_mount.stl'):
        check_head_dir(str(folder))


def test_missing_link(tmp_path):
    folder = write_synthetic_head_dir(tmp_path / 'h')
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    del doc['links']['head_mount']
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    with pytest.raises(HeadDirError, match='head_mount'):
        check_head_dir(str(folder))


# --- review M5: conduit position args that hcad would ignore ---------------------------------------------------

def test_conduit_position_args_refused_when_head_has_cable_exit(tmp_path):
    from awm_bringup.cell_config import CONDUIT_POSITION_ARGS, check_conduit_args, conduit_source
    from head_fixture import write_map_override_head_dir
    explicit = write_synthetic_head_dir(tmp_path / 'e')
    rear = write_map_override_head_dir(tmp_path / 'r')
    assert conduit_source(str(explicit)) == 'explicit' and conduit_source(str(rear)) == 'rear_estimate'
    for folder in (explicit, rear):
        check_conduit_args('hcad', str(folder), [])                        # nothing set: fine
        check_conduit_args('hcad', str(folder), ['conduit_dia_mm'])        # dia / len apply to hcad
        for name in CONDUIT_POSITION_ARGS:
            with pytest.raises(ValueError, match=name):
                check_conduit_args('hcad', str(folder), [name])
    check_conduit_args('hbox', '', list(CONDUIT_POSITION_ARGS))           # H-BOX uses them


def test_conduit_position_args_allowed_for_parametric_hcad(tmp_path):
    """A head without cable exit uses the parametric conduit (review I4): the position args are used, not refused."""
    from awm_bringup.cell_config import CONDUIT_POSITION_ARGS, check_conduit_args, conduit_source
    folder = write_synthetic_head_dir(tmp_path / 'p')
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    doc.pop('cable_exit_tool0_mm')
    doc.pop('cable_axis_tool0')
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    assert conduit_source(str(folder)) == 'parametric'
    check_conduit_args('hcad', str(folder), list(CONDUIT_POSITION_ARGS))
