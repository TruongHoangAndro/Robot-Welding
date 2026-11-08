import { describe, expect, it } from 'vitest';
import { defaultUpAxis, tcpRows } from '../src/lib/cellinfo';

describe('S1 up-axis default (audit m6)', () => {
  it('uses the cell up axis when known, else +Z', () => {
    expect(defaultUpAxis('+Y')).toBe('+Y');
    expect(defaultUpAxis('-X')).toBe('-X');
    expect(defaultUpAxis(null)).toBe('+Z');
    expect(defaultUpAxis('Y')).toBe('+Z');
  });
});

describe('S10 taught TCP rows (audit m4)', () => {
  it('shows tool0 position, beam axis with its angle to tool0 +Z and the teach record', () => {
    const rows = Object.fromEntries(
      tcpRows({
        method: 'teach_ui',
        xyz_mm: [43.8288, 0, 279.6056],
        quat_xyzw: [0, 0.5, 0, 0.8660254],
        beam_axis_tool0: [0.8660254, 0, 0.5],
        component: 'LightWELD:1',
        edge_id: 'E3955',
        reversed: false,
        stickout_mm: 0,
        mouth_radius_mm: 2.5,
        taught_by: 'adm',
        taught_at: '2026-11-07T12:25:04Z',
      }),
    );
    expect(rows['method']).toBe('teach_ui');
    expect(rows['tool0 → TCP mm']).toBe('43.83, 0.00, 279.61');
    expect(rows['beam axis (tool0)']).toContain('60.0° from tool0 +Z');
    expect(rows['picked']).toBe('LightWELD:1 · E3955 · mouth r 2.50 mm');
    expect(rows['stick-out mm']).toBe('0.00');
    expect(rows['taught']).toBe('adm at 2026-11-07T12:25:04Z');
  });
  it('mapped TCP without a teach record shows what is known', () => {
    const rows = tcpRows({ method: null, xyz_mm: [0, 0, 100], quat_xyzw: null, beam_axis_tool0: null });
    expect(rows.map((r) => r[0])).toEqual(['method', 'tool0 → TCP mm']);
  });
});

describe('S1 up-axis note (review minor 7)', () => {
  it('says the default comes from the cell, or why it does not', async () => {
    const { upAxisNote } = await import('../src/lib/cellinfo');
    expect(upAxisNote('+Y', '+Y', false, null)).toBe('default from the cell (+Y)');
    expect(upAxisNote('+Y', '+Z', true, null)).toBe('changed (cell up axis +Y)');
    expect(upAxisNote(null, '+Z', false, null)).toBe('no cell STEP imported: default +Z');
    expect(upAxisNote(null, '+Z', false, 'HTTP 500')).toBe('cell up axis not available (HTTP 500): default +Z');
  });
});
