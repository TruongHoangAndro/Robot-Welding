import { describe, expect, it } from 'vitest';
import { cadBlocker, cadGate, check4Statics, overrideBanner, type CadCheck, type OverrideState } from '../src/lib/cadcheck';

const c4hits = { hits: [{ static: 'Jig:1' }, { static: 'Timing Belts:1' }, { static: 'Jig:1' }] };
const red4: CadCheck = {
  status: 'red',
  phi_deg: 0,
  checks: [
    { id: 1, name: 'positioner match', status: 'green' },
    { id: 4, name: 'swept rotating set vs static', status: 'red', values: c4hits, note: 'static object(s) …' },
  ],
};
const red14: CadCheck = { ...red4, checks: [{ id: 1, name: 'positioner match', status: 'red', note: 'radius differs' }, red4.checks[1]] };
const ov = (valid: boolean, problem: string | null = null): OverrideState => ({
  valid,
  problem,
  override: {
    by: 'adm',
    created_at: '2026-11-07T12:00:00Z',
    reason: 'OI-23: jig not separated',
    findings: ['Jig:1', 'Timing Belts:1'],
    check_id: 4,
    scope: 'kinematic_planning',
    cad_check_sha256: 'a'.repeat(64),
  },
});

describe('CP-62 S2 gate states', () => {
  it('check 4 statics: unique names from values.hits, else empty', () => {
    expect(check4Statics(red4)).toEqual(['Jig:1', 'Timing Belts:1']);
    expect(check4Statics({ ...red4, checks: [red4.checks[0]] })).toEqual([]);
  });

  it('red check 4 only, no override: blocked with the CAD cause (OI-23, S10 roles), Admin may override', () => {
    const g = cadGate(true, red4, 'ok', null);
    expect(g.block).toMatch(/Jig:1, Timing Belts:1/);
    expect(g.block).toMatch(/OI-23/);
    expect(g.block).toMatch(/S10/);
    expect(g.block).not.toMatch(/axis pick/);
    expect(g.canOverride).toBe(true);
    expect(g.overridden).toBeNull();
  });

  it('valid override: not blocked, banner names admin, time and reason', () => {
    const g = cadGate(true, red4, 'ok', ov(true));
    expect(g.block).toBeNull();
    expect(g.overridden).toBe('Overridden for kinematic planning by adm at 2026-11-07T12:00:00Z: OI-23: jig not separated');
    expect(g.canOverride).toBe(false);
    expect(overrideBanner(ov(true).override)).toBe(g.overridden);
  });

  it('void override (check changed): blocked, says why, Admin may record it again', () => {
    const g = cadGate(true, red4, 'ok', ov(false, 'the cross-check changed after the override was recorded'));
    expect(g.block).toMatch(/void/);
    expect(g.block).toMatch(/changed/);
    expect(g.canOverride).toBe(true);
    expect(g.overridden).toBeNull();
  });

  it('another red check: blocked by that check; no override offered even with a record', () => {
    const g = cadGate(true, red14, 'ok', ov(true));
    expect(g.block).toMatch(/check 1: radius differs/);
    expect(g.block).toMatch(/roles or the positioner-axis pick/);
    expect(g.canOverride).toBe(false);
    expect(g.overridden).toBeNull();
  });

  it('stale check: blocked (re-run) even with a valid override; no override offered', () => {
    const g = cadGate(true, red4, 'stale', ov(true));
    expect(g.block).toMatch(/re-run/);
    expect(g.canOverride).toBe(false);
  });

  it('green / yellow: open; cadBlocker keeps its old signature', () => {
    expect(cadGate(true, { ...red4, status: 'green', checks: [] }, 'ok', null).block).toBeNull();
    expect(cadBlocker(true, red4, 'ok')).toMatch(/Jig:1/);
    expect(cadBlocker(true, red4, 'ok', ov(true))).toBeNull();
    expect(cadBlocker(false, null, null)).toBeNull();
  });
});
