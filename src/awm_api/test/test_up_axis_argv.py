"""``--up-axis`` must reach the geometry CLI as a single token even for the negative axes.

argparse reads a value that begins with ``-`` as a new option, so the two-token form
``['--up-axis', '-Z']`` fails with ``error: argument --up-axis: expected one argument``.
Every command built in :mod:`awm_api.adapters` must therefore use the joined form
``--up-axis=<value>``. Regression test for the S2 assembly-import crash of 2026-11-11, where an
import with a negative up axis stopped at ``export-view`` before the D20 view model was written.
"""

import argparse

import pytest

from awm_api import adapters

AXES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']


def _up_axis(argv) -> str:
    """Parse ``--up-axis`` from a built argv exactly as the geometry CLI argparse would.

    Raises ``SystemExit`` (via ``parser.error``) when the value is passed as its own token and
    starts with ``-`` — the bug under test.
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--up-axis', choices=AXES)
    namespace, _ = parser.parse_known_args([str(a) for a in argv])
    return namespace.up_axis


def _step(steps, label):
    return next(s for s in steps if s.label == label)


@pytest.mark.parametrize('axis', AXES)
def test_import_steps_export_view_up_axis(settings, tmp_path, axis):
    steps = adapters.import_steps(settings, tmp_path / 'run', tmp_path / 'model.step',
                                  '564082', 'B', None, axis, components=True)
    assert _up_axis(_step(steps, 'export-view').argv) == axis


@pytest.mark.parametrize('axis', AXES)
def test_cell_import_steps_up_axis(settings, tmp_path, axis):
    steps = adapters.cell_import_steps(settings, tmp_path / 'cell.step', tmp_path / 'out', axis)
    assert _up_axis(_step(steps, 'export-cell-view').argv) == axis


@pytest.mark.parametrize('axis', AXES)
def test_component_view_steps_up_axis(settings, tmp_path, axis):
    steps = adapters.component_view_steps(settings, tmp_path / 'm.step', 'comp:1', axis, tmp_path / 'out')
    assert _up_axis(_step(steps, 'export-view').argv) == axis


@pytest.mark.parametrize('axis', AXES)
def test_pick_circle_steps_up_axis(settings, tmp_path, axis):
    steps = adapters.pick_circle_steps(settings, tmp_path / 'm.step', 'E21', axis,
                                       tmp_path / 'pick.json', components=True)
    assert _up_axis(_step(steps, 'pick-circle').argv) == axis
