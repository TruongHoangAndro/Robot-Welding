"""WP-S S.6 Task 2: pure parts of the Mode C runner — per-θ verdicts, C9 summary, compressed θ table."""

import pytest

from awm_path.spike_s6 import compress_rows, theta_summary


def _row(theta, f4=True, lb=10.0, closest='part<->head_body', contact=True, reason=None, pairs=()):
    return {'theta_deg': theta, 'f4_valid': f4, 'f4_contacts': list(pairs), 'clearance_lb_mm': lb,
            'closest_pair': closest, 'contact_ok': contact, 'contact_reason': reason, 'offending_faces': []}


def test_theta_summary_c9_verdict_classes_and_min_clearance():
    rows = [_row(0.0), _row(-1.0, lb=5.0, closest='part3_1<->head_mount'), _row(-2.0, lb=None),
            _row(-3.0, f4=False, pairs=['part<->nozzle_body']), _row(-4.0, contact=False,
                                                                     reason='other_face_within_standoff'),
            _row(-5.0)]
    s = theta_summary(rows, weld_clearance_mm=5.0, step_deg=1.0, r_max_mm=100.0)
    assert s['C9_pass'] is False and s['states'] == 6 and s['states_ok'] == 3
    assert s['min_clearance_lb_mm'] is None                       # one angle below the lowest level
    assert s['failing']['clearance'] == [{'theta_from_deg': -2.0, 'theta_to_deg': -2.0, 'states': 1}]
    assert s['failing']['f4'][0]['theta_from_deg'] == -3.0
    assert s['failing']['contact'][0]['theta_from_deg'] == -4.0
    assert [r['ok'] for r in rows] == [True, True, False, False, False, True]
    assert s['f4_pairs'] == {'part<->nozzle_body': 1}
    assert s['closest_pairs_at_min'] == {'part<->head_body': 1}       # below the lowest level counts as smallest
    ok = theta_summary([_row(0.0, lb=7.5, closest='a<->b'), _row(-1.0, lb=10.0)], 5.0, 1.0, 100.0)
    assert ok['C9_pass'] is True and ok['min_clearance_lb_mm'] == 7.5 and ok['theta_at_min_deg'] == [0.0]
    assert ok['between_states_min_mm'] == pytest.approx(7.5 - 100.0 * 0.017453292519943295 / 2)


def test_compress_rows_merges_runs_with_the_same_verdict():
    rows = [_row(0.0), _row(-1.0), _row(-2.0, lb=5.0), _row(-3.0, lb=5.0), _row(-4.0)]
    for r in rows:
        r['ok'] = True
    runs = compress_rows(rows)
    assert [(r['theta_from_deg'], r['theta_to_deg'], r['states'], r['clearance_lb_mm']) for r in runs] == [
        (0.0, -1.0, 2, 10.0), (-2.0, -3.0, 2, 5.0), (-4.0, -4.0, 1, 10.0)]


def test_s6_head_assumptions_come_from_the_description():
    """Review minor 6: the S.6 record names the head of the description in use, not TST-002 §3.1 for every head."""
    from awm_path.core.contact import ContactParams
    from awm_path.spike_s6 import head_assumptions
    head = {'nozzle_radius_mm': 6.0, 'contact_standoff_mm': 0.25, 'nozzle_tip_length_mm': 5.0,
            'tool0_T_tcp_mm': {'position_mm': [43.8, 0.0, 279.6], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}}
    lines = head_assumptions('H-CAD', ContactParams(nozzle_radius_mm=6.0, standoff_mm=0.25), head)
    text = ' '.join(lines)
    assert 'H-CAD head' in text and 'Ø12 mm' in text and '(43.8, 0, 279.6)' in text
    assert 'TST-002 §3.1' not in text and "rule (b) 'rising'" in text
