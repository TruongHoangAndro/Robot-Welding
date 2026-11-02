"""Environment gate for the M4 core (WP-S S.2): ur-analytic-ik is installed in /opt/awm_venv at the pinned version."""
import importlib.metadata


def test_ur_analytic_ik_version_is_pinned() -> None:
    """The analytic UR IK library is importable and pinned (a missing library is a failure, never a skip)."""
    assert importlib.metadata.version('ur_analytic_ik') == '0.1.0.post3'
    from ur_analytic_ik import ur5e  # noqa: F401
