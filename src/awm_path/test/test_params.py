"""ChainParams defaults are the D16 file values (one effective definition)."""

from dataclasses import asdict
from pathlib import Path

import pytest

from awm_path.params import ChainParams, from_yaml


def _params_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_bringup')) / 'config' / 'awm_params.yaml'


def test_yaml_matches_defaults():
    loaded = asdict(from_yaml(_params_file()))
    defaults = asdict(ChainParams())
    assert set(loaded) == set(defaults)
    for key, value in defaults.items():
        assert loaded[key] == pytest.approx(value) if isinstance(value, (float, list)) \
            else loaded[key] == value, key


def test_unknown_key_rejected(tmp_path):
    bad = tmp_path / 'p.yaml'
    bad.write_text('/**:\n  ros__parameters:\n    speed_mm_s: 3\n')
    with pytest.raises(KeyError, match='speed_mm_s'):
        from_yaml(bad)
