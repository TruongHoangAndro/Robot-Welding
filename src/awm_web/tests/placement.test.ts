import { describe, expect, it } from 'vitest';
import { emptyPlacementForm, placementRequest } from '../src/lib/placement';
import { parseRoute } from '../src/lib/route';

describe('placement request (S2 panel)', () => {
  it('manual pose in mm / deg', () => {
    const f = { ...emptyPlacementForm(), position_mm: ['10', '-20.5', '300'], rpy_deg: ['90', '0', '-45'] };
    expect(placementRequest(f, 'P1_A')).toEqual({ body: { flange_T_part: { position_mm: [10, -20.5, 300], rpy_deg: [90, 0, -45] } } });
  });
  it('refuses empty, non-numeric and implausible values', () => {
    const f = { ...emptyPlacementForm(), position_mm: ['', 'x', '9000'], rpy_deg: ['0', 'NaN', '0'] };
    const r = placementRequest(f, 'P1_A');
    expect('errors' in r && r.errors).toEqual(['position x: a number', 'position y: a number', 'position z: |value| ≤ 5000 mm', 'pitch: a number']);
  });
  it('copy from another job only', () => {
    const f = { ...emptyPlacementForm(), mode: 'copy' as const, copy_from: ' 564082_A ' };
    expect(placementRequest(f, 'P1_A')).toEqual({ body: { copy_from: '564082_A' } });
    expect('errors' in placementRequest({ ...f, copy_from: 'P1_A' }, 'P1_A')).toBe(true);
    expect('errors' in placementRequest({ ...f, copy_from: '../x' }, 'P1_A')).toBe(true);
  });
});

describe('S4 route', () => {
  it('#/jobs/<id>/sim', () => {
    expect(parseRoute('#/jobs/564082_A/sim', '')).toEqual({ page: 'sim', jobId: '564082_A' });
    expect(parseRoute('#/jobs/564082_A', '')).toEqual({ page: 'job', jobId: '564082_A' });
    expect(parseRoute('#/jobs/bad/sim', '')).toEqual({ page: 'jobs' });
  });
});
