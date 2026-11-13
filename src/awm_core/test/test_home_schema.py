"""CP-64: D22 cell home schema (taught joint values, deg)."""
import copy
import json

import pytest

from awm_core import jsonio
from awm_core.jsonio import SchemaValidationError

EXAMPLE = jsonio.SCHEMA_DIR / 'examples' / 'home.json'


def example():
    return copy.deepcopy(json.loads(EXAMPLE.read_text()))


def test_valid_home_accepted():
    jsonio.validate(example(), 'home')


def test_missing_positioner_rejected():
    doc = example()
    del doc['positioner_joint_deg']
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'home')


def test_extra_joint_rejected():
    doc = example()
    doc['robot_joints_deg']['extra_joint'] = 0.0
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'home')


def test_source_must_be_taught():
    doc = example()
    doc['source'] = 'nominal'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'home')
