"""S.5c Task 0 step 2 (CP-56 gap): the IK TCP offset and the F9 nozzle equal the cell description in use.

Renders ``awm_description/urdf/awm_cell_v2.urdf.xacro`` (H-BOX) with xacro and compares
:func:`awm_path.core.head.head_from_urdf` with an independent chain from ``urdf_parser_py`` (test-only FK) and with the
xacro args (one source: the args of the description, CP-56 decision 2). No expected offset is a constant here: the
expected values are computed from the args passed to xacro.
"""
import subprocess
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

from awm_core import units
from awm_path.core.contact import ContactParams
from awm_path.core.head import contact_params_for, head_from_urdf

from test_core_kinematics_urdf import _origin_mm  # noqa: E402  (test-only URDF origin -> mm matrix)


def _render(**args) -> str:
    xacro = Path(get_package_share_directory('awm_description')) / 'urdf' / 'awm_cell_v2.urdf.xacro'
    return subprocess.run(['xacro', str(xacro)] + [f'{k}:={v}' for k, v in args.items()], check=True,
                          capture_output=True, text=True).stdout


def _urdf_chain_mm(xml: str, root: str, tip: str) -> np.ndarray:
    robot = URDF.from_xml_string(xml)
    joints = {j.name: j for j in robot.joints}
    t = np.eye(4)
    for name in robot.get_chain(root, tip, joints=True, links=False):
        assert joints[name].type == 'fixed'
        t = t @ _origin_mm(joints[name])
    return t


@pytest.mark.parametrize('args', [{}, {'mount_h_mm': 55, 'head_h_mm': 300, 'nozzle_len_mm': 30,
                                       'nozzle_dia_mm': 12, 'contact_standoff_mm': 0.25},
                                  {'nozzle_dia_mm': 8}])
def test_head_from_description_equals_the_urdf_chain_and_the_xacro_args(args) -> None:
    xml = _render(**args)
    head = head_from_urdf(xml)
    ref = _urdf_chain_mm(xml, 'tool0', 'tcp_laser')
    assert np.allclose(head.tool0_T_tcp_mm, ref, atol=1e-9)
    # expected from the args actually rendered (xacro defaults where not given; TST-002 §3.1 values)
    mount_h, head_h, noz_len = (float(args.get(k, d)) for k, d in
                                (('mount_h_mm', 60), ('head_h_mm', 381), ('nozzle_len_mm', 40)))
    assert head.tool0_T_tcp_mm[:3, 3] == pytest.approx([0.0, 0.0, mount_h + head_h + noz_len], abs=1e-9)
    assert head.nozzle_radius_mm == pytest.approx(float(args.get('nozzle_dia_mm', 20)) / 2.0, abs=1e-9)
    assert head.contact_standoff_mm == pytest.approx(float(args.get('contact_standoff_mm', 0.3)), abs=1e-9)
    p = contact_params_for(head, ContactParams())
    assert p.nozzle_radius_mm == head.nozzle_radius_mm and p.standoff_mm == head.contact_standoff_mm


def test_default_description_matches_the_contact_params_defaults() -> None:
    """The ContactParams defaults (used by unit tests) equal the H-BOX description defaults (no second source in
    the runners: they always build the params from the description)."""
    head = head_from_urdf(_render())
    d = ContactParams()
    assert (d.nozzle_radius_mm, d.standoff_mm) == pytest.approx((head.nozzle_radius_mm, head.contact_standoff_mm))
    assert units.mm_to_m(head.tool0_T_tcp_mm[2, 3]) > 0.0
