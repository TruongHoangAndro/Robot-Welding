"""WP-S S.5b Task 3: pure parts of the Mode B screening runner — valid intervals, failure summary, coverage map,
data-file checks."""

import pytest

from awm_path.spike_s5b import coverage_map, failure_summary, read_input, valid_intervals


def test_valid_intervals_in_mm_with_wrap():
    flags = [True, False, False, True, True]
    assert valid_intervals(flags, pitch_mm=2.0) == [{'s_from_mm': 6.0, 's_to_mm': 10.0, 'points': 3}]
    assert valid_intervals([False] * 3, 1.0) == []


def test_failure_summary_counts_classes_pairs_and_runs():
    pts = [{'point': 0, 'pairs': {}}, {'point': 1, 'pairs': {'part<->head_body': 3, 'a<->b': 1}},
           {'point': 2, 'pairs': {'part<->head_body': 2}}, {'point': 3, 'pairs': {}}]
    classes = [None, 'F4', 'F4', 'IK']
    s = failure_summary(pts, classes, pitch_mm=0.5)
    assert s['class_counts'] == {'F4': 2, 'IK': 1}
    assert s['dominant_class'] == 'F4'
    assert s['top_pairs'] == {'part<->head_body': 5, 'a<->b': 1}
    assert s['runs'] == [{'class': 'F4', 'points': 2, 's_from_mm': 0.5, 's_to_mm': 1.0,
                          'pairs': {'part<->head_body': 5, 'a<->b': 1}},
                         {'class': 'IK', 'points': 1, 's_from_mm': 1.5, 's_to_mm': 1.5, 'pairs': {}}]
    none = failure_summary(pts, [None] * 4, 0.5)
    assert none['dominant_class'] is None and none['runs'] == []


def test_coverage_map_merges_bins_with_the_same_angle_set():
    bins = {0.0: [True, True, False, False], 30.0: [False, True, True, False]}
    m = coverage_map(bins, bin_mm=1.0, length_mm=3.5)
    assert m == [{'s_from_mm': 0.0, 's_to_mm': 1.0, 'thetas': [0.0]},
                 {'s_from_mm': 1.0, 's_to_mm': 2.0, 'thetas': [0.0, 30.0]},
                 {'s_from_mm': 2.0, 's_to_mm': 3.0, 'thetas': [30.0]},
                 {'s_from_mm': 3.0, 's_to_mm': 3.5, 'thetas': []}]


def test_read_input_checks_the_data_file(tmp_path):
    p = tmp_path / 'in.yaml'
    p.write_text('seam_id: S02\nseam_file: spike_seam.json\ntheta_step_deg: 30\npsi_cases: [free, wire]\n'
                 'pitch_rule: scene_valid\npitch_min_mm: 0.05\ncoverage_bin_mm: 1.0\nset_sizes: [2, 3]\n'
                 'join_overlap_mm: null\nwindow_start_stride: 1\nmax_sets_listed: 10\ntime_budget_s: 7200\n'
                 'b03_margin_mm: 3.0\n')
    d = read_input(p)
    assert d['theta_step_deg'] == 30.0 and d['psi_cases'] == ['free', 'wire'] and d['join_overlap_mm'] is None
    p.write_text(p.read_text().replace('scene_valid', 'kinematics'))
    with pytest.raises(ValueError):
        read_input(p)


def test_write_md_renders_a_minimal_record(tmp_path):
    from awm_path.spike_s5b import write_md
    row = {'theta_deg': 0.0, 'equivalent_thetas_deg': [-360.0, 0.0, 360.0],
           'pitch': {'points': 10, 'pitch_mm': 1.0, 'ok': True}, 'valid_node_pct': 50.0,
           'valid_intervals': [{'s_from_mm': 0.0, 's_to_mm': 4.0, 'points': 5}],
           'longest_window': {'s_from_mm': 0.0, 's_to_mm': 4.0, 'length_mm': 4.0, 'pct': 40.0, 'label': 'S+E+W+'},
           'failures': failure_summary([{'pairs': {'a<->b': 1}}] * 2, ['F4', None], 1.0),
           'timing_s': {'total_s': 1.0}}
    rec = {'head_model': 'H-BOX', 'commit': 'abc', 'dirty': False, 'created_at': 'now',
           'scene_backend': {'scene_hash': 'h', 'content': {'backend': 'fake'}}, 'kinematics_backend': 'k',
           'clearance_links': ['head_body'], 'clearance_method': 'm', 'seam': {'seam_id': 'S', 'length_mm': 10.0},
           'pitch_rule': 'scene_valid', 'input': {'pitch_min_mm': 0.05, 'window_start_stride': 1,
                                                  'theta_step_deg': 30.0, 'coverage_bin_mm': 1.0},
           'join_overlap_mm': 2.0, 'positioner_limits': {'source': 'x.yaml', 'lower_deg': -400.0, 'upper_deg': 400.0},
           'thetas_deg': [0.0], 'theta_classes': {'0': [0.0]},
           'theta_equivalence': {'max_flange_diff_mm': 0.0, 'max_flange_diff_deg': 0.0},
           'cases': {'free': {'psi_deg': [0.0, 10.0], 'thetas': [row],
                              'coverage_map': coverage_map({0.0: [True, False]}, 5.0, 10.0),
                              'bins_covered_any_pct': 50.0,
                              'cover_sets': {'2': {'count': 0, 'sets': [], 'best': None},
                                             '3': {'count': 1, 'sets': [{'thetas': [0.0, 30.0, 60.0],
                                                                         'min_join_overlap_mm': 2.5,
                                                                         'segments': [{'theta_deg': 0.0,
                                                                                       's_from_mm': 0.0,
                                                                                       's_to_mm': 5.0}]}],
                                                   'best': None}}}},
           'not_run': ['x'], 'assumptions': ['y']}
    p = tmp_path / 'r.md'
    write_md(p, rec)
    text = p.read_text()
    assert 'Screening only' in text and 'suggestions' in text and 'θ 0° / 30° / 60°' in text
    assert 'no set covers 100 %' in text


def test_rotating_set_ids_from_cell_manifest_and_placement():
    from awm_path.spike_s5b import rotating_set_ids
    cell = {'objects': [{'object_id': 'jig_1', 'parent_frame': 'world'},
                        {'object_id': 'part2_1', 'parent_frame': 'positioner_flange'}]}
    placement = {'objects': [{'object_id': 'part', 'parent_frame': 'positioner_flange'}]}
    assert rotating_set_ids(cell, placement) == {'part', 'part2_1'}


def test_theta_invariant_blocking_marks_bins_blocked_for_every_psi():
    from awm_path.spike_s5b import theta_invariant_blocking
    head, rot = {'head_body', 'nozzle_tip'}, {'part', 'holder'}
    rows = [  # (s_mm, psi_deg, kinematic reason, scene_contacts) from all θ
        (0.5, 0.0, None, 'part<->nozzle_tip'), (0.5, 10.0, None, 'holder<->head_body;jig<->forearm_link'),
        (1.5, 0.0, None, 'part<->nozzle_tip'), (1.5, 10.0, None, 'jig<->head_body'),   # ψ 10 free of the set
        (2.5, 0.0, None, 'part<->nozzle_tip'),                                         # ψ 10 never seen
        (3.2, 0.0, 'no_ik_unreachable', ''), (3.4, 10.0, None, ''),
    ]
    r = theta_invariant_blocking(rows, psi_all=[0.0, 10.0], head_links=head, rotating_ids=rot, bin_mm=1.0,
                                 length_mm=4.0)
    assert [b['blocked'] for b in r['bins']] == [True, False, False, False]
    assert r['bins'][2]['psi_unseen'] == [10.0]
    assert r['blocked_runs'] == [{'s_from_mm': 0.0, 's_to_mm': 1.0}]
    assert r['pairs'] == {'holder<->head_body': 1, 'part<->nozzle_tip': 3}


def test_theta_invariant_blocking_needs_every_psi_at_every_point_of_the_bin():
    """Review M3: ψ seen at different points of a bin is no evidence that one point is blocked for every ψ."""
    from awm_path.spike_s5b import theta_invariant_blocking
    head, rot = {'head_body'}, {'part'}
    rows = [(2.0, 0.0, None, 'part<->head_body'), (8.0, 10.0, None, 'part<->head_body')]
    r = theta_invariant_blocking(rows, psi_all=[0.0, 10.0], head_links=head, rotating_ids=rot, bin_mm=10.0,
                                 length_mm=10.0)
    assert r['bins'][0]['blocked'] is False
    assert r['bins'][0]['psi_unseen'] == [0.0, 10.0]
    # both ψ at both points, all contacting → blocked
    rows += [(2.0, 10.0, None, 'part<->head_body'), (8.0, 0.0, None, 'part<->head_body')]
    r = theta_invariant_blocking(rows, psi_all=[0.0, 10.0], head_links=head, rotating_ids=rot, bin_mm=10.0,
                                 length_mm=10.0)
    assert r['bins'][0]['blocked'] is True and r['bins'][0]['psi_unseen'] == []
    # a point with only kinematically invalid candidates has no evidence → not blocked
    rows.append((5.0, 0.0, 'no_ik_unreachable', ''))
    r = theta_invariant_blocking(rows, psi_all=[0.0, 10.0], head_links=head, rotating_ids=rot, bin_mm=10.0,
                                 length_mm=10.0)
    assert r['bins'][0]['blocked'] is False
