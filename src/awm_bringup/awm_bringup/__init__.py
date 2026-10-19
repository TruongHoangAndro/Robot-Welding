"""APLSMN601 bring-up helpers: locations of the D16 configuration and the repository template."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def share_dir() -> Path:
    return Path(get_package_share_directory('awm_bringup'))


def params_file() -> Path:
    """D16 parameters file (single configuration source)."""
    return share_dir() / 'config' / 'awm_params.yaml'


def repo_template() -> Path:
    return share_dir() / 'repo_template'
