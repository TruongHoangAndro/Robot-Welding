"""YAML subset reader / writer (no OCC; PyYAML is used as a cross-check only where it is installed)."""

import importlib.util

import pytest

from awm_geometry import miniyaml

MAP_TEXT = """\
# comment line
schema: cad_map/0.1
source:
  file: docs/3D/APLSMN601_Automated Weldment System.step
  sha256: bc66c09ef1878d0e7389cf852b695d8b59917994a98d5739e67ba627bab7e824
  unit: mm
  up_axis: +Y            # trailing comment; '+Y' stays a string
match: instance_name
default: deny
frames:
  world:
    origin_from: ur_base
  ur_base:
    component: 'UR5E/UR7E:1'
    note: origin on top of pedestal Part5 (file Y 518.40); identity rotation
  positioner_axis:
    point_file_mm: [-102.03, 593.40, 321.58]   # comment
    direction_file: [1.0, 0.0, 0.0]
roles:
  part:
    - '564082-Assembly-3 parts:2'           # golden part
  head_cad:
    - path: 'ASSY, Welding tool:1'
      depth: 1
  ignore:
    - 'UR5E/UR7E:1'
  empty_list: []
  quoted: "a # not a comment"
  hash_in_single: 'it''s #1'
  nothing:
  flag: true
"""

EXPECTED = {
    'schema': 'cad_map/0.1',
    'source': {'file': 'docs/3D/APLSMN601_Automated Weldment System.step',
               'sha256': 'bc66c09ef1878d0e7389cf852b695d8b59917994a98d5739e67ba627bab7e824',
               'unit': 'mm', 'up_axis': '+Y'},
    'match': 'instance_name', 'default': 'deny',
    'frames': {'world': {'origin_from': 'ur_base'},
               'ur_base': {'component': 'UR5E/UR7E:1',
                           'note': 'origin on top of pedestal Part5 (file Y 518.40); identity rotation'},
               'positioner_axis': {'point_file_mm': [-102.03, 593.40, 321.58],
                                   'direction_file': [1.0, 0.0, 0.0]}},
    'roles': {'part': ['564082-Assembly-3 parts:2'],
              'head_cad': [{'path': 'ASSY, Welding tool:1', 'depth': 1}],
              'ignore': ['UR5E/UR7E:1'], 'empty_list': [], 'quoted': 'a # not a comment',
              'hash_in_single': "it's #1", 'nothing': None, 'flag': True},
}


def test_map_subset_parses():
    assert miniyaml.loads(MAP_TEXT) == EXPECTED


def test_same_result_as_pyyaml_where_installed():
    if importlib.util.find_spec('yaml') is not None:  # ROS interpreter: cross-check
        import yaml
        assert yaml.safe_load(MAP_TEXT) == EXPECTED


def test_apostrophe_in_plain_scalar_and_comments():
    """A quote opens a quoted token only at the start of a token; '#' after a space outside quotes is a comment."""
    text = "a: robot's base # c\nb: 'it''s' # c\nc: [x, 'y # z'] # c\nd: -Y\ne: -1.5\n"
    data = {'a': "robot's base", 'b': "it's", 'c': ['x', 'y # z'], 'd': '-Y', 'e': -1.5}
    assert miniyaml.loads(text) == data
    if importlib.util.find_spec('yaml') is not None:
        import yaml
        assert yaml.safe_load(text) == data


def test_round_trip():
    data = {'world_T_base': {'position_mm': [0.0, 0.0, 0.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0],
                             'note': 'Assumption: x: y # z'},
            'list_of_maps': [{'a': 1, 'b': [1, 2]}, {'c': 'd'}], 'n': None, 'ok': False}
    text = miniyaml.dumps(data)
    assert miniyaml.loads(text) == data
    if importlib.util.find_spec('yaml') is not None:
        import yaml
        assert yaml.safe_load(text) == data


@pytest.mark.parametrize('text', [
    'a: &x 1', 'a: {b: 1}', 'a: |\n  x', 'a:\n  - [1, [2]]', 'a: 1\n   b: 2', 'a: 1\na: 2', 'a: \'x',
    'a: [1, ]', 'a: [1,,2]', 'a: [, 1]',                       # empty flow items
    'a: [1,2]]', 'a: [1, 2}]', 'a: x]', 'a: x}',               # stray closing brackets
    'a: - b', 'a: ? b', 'a: , b', 'a: ] b', 'a: } b', 'a: ?',  # plain scalars starting with an indicator
    '---\na: 1\n---\nb: 2', 'a: 1\n---',                     # a second document
    "a: 'x' 'y'", 'a: "x" "y"', "a: 'x'y",                    # text after a closing quote
    '? a\n: 1', '?', 'a:\n  ? b',                             # complex keys
])
def test_unsupported_or_malformed_fails(text):
    with pytest.raises(miniyaml.YamlSubsetError):
        miniyaml.loads(text)
