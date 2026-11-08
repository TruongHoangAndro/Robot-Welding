"""Session fixture ``golden`` (G1.22): the golden repository of part 564082, rebuilt from the cell STEP alone by
``scripts/golden_rebuild.py`` (once per session, cached and locked under ``build/golden_repo`` or
``AWM_GOLDEN_OUT``). Skipped with ``AWM_CELL_STEP not set`` when the cell STEP is not given (the only allowed skip).
The fixture value is the rebuild summary (``golden_summary.json``): ``repo``, ``job_id``, ``joints`` (J1 / J2 joint
edge ids), ``program`` …
"""
import importlib.util
from pathlib import Path

import pytest


def _golden_rebuild():
    path = Path(__file__).resolve().parents[3] / 'scripts' / 'golden_rebuild.py'
    spec = importlib.util.spec_from_file_location('golden_rebuild', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='session')
def golden():
    return _golden_rebuild().pytest_golden()
