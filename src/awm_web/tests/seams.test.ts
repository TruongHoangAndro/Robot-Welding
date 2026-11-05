import { describe, expect, it } from 'vitest';
import {
  arcBoundaries,
  newPick,
  nextSeamId,
  picksFromDoc,
  seamFrames,
  setArcs,
  setMode,
  validatePicks,
  type SeamDocSeam,
} from '../src/lib/seams';

/** Closed circle r = 10 mm in the XY plane, Z up, sampled every 1 mm of arc; n1 = n2 = +Z (out of the material). */
function ring(): SeamDocSeam {
  const r = 10;
  const L = 2 * Math.PI * r;
  const n = Math.round(L);
  const points = [];
  for (let i = 0; i <= n; i++) {
    const s = (L * i) / n;
    const a = s / r;
    points.push({
      s_mm: s,
      p_mm: [r * Math.cos(a), r * Math.sin(a), 0] as [number, number, number],
      t: [-Math.sin(a), Math.cos(a), 0] as [number, number, number],
      n1: [0, 0, 1] as [number, number, number],
      n2: [0, 0, 1] as [number, number, number],
    });
  }
  return {
    seam_id: 'S01', edges: ['E1'], joint_edge_id: 'JE1', reversed: false, closed: true, length_mm: L,
    joint_type: 'butt', sequence: 1, arcs: 2, arc_overlap_mm: 2, positioner_mode: 'sync',
    positioner_angles_deg: null, start_offset_mm: 0, geometry: { spacing_mm: L / n, points },
  };
}

describe('seam ids and defaults', () => {
  it('next id fills from S01 and keeps two digits', () => {
    expect(nextSeamId([])).toBe('S01');
    expect(nextSeamId(['S01', 'S02'])).toBe('S03');
    expect(nextSeamId(['S01', 'S09'])).toBe('S10');
    expect(nextSeamId(['S99'])).toBe('S100');
  });
  it('new pick has Goal 1 defaults (sync, one arc, no angles)', () => {
    const p = newPick('S01', 'E7', 1);
    expect(p).toMatchObject({ seam_id: 'S01', edges: ['E7'], reversed: false, joint_type: 'butt', sequence: 1,
      arcs: 1, positioner_mode: 'sync', positioner_angles_deg: null, start_offset_mm: 0 });
    expect(p.arc_overlap_mm).toBeGreaterThanOrEqual(0);
  });
});

describe('positioner mode and arcs', () => {
  it('mode B gets one angle per arc, other modes none', () => {
    let p = setArcs(newPick('S01', 'E1', 1), 3);
    p = setMode(p, 'B');
    expect(p.positioner_angles_deg).toEqual([0, 120, 240]);
    p = setArcs(p, 2);
    expect(p.positioner_angles_deg).toEqual([0, 120]);
    p = setArcs(p, 4);
    expect(p.positioner_angles_deg).toHaveLength(4);
    expect(setMode(p, 'C').positioner_angles_deg).toBeNull();
    expect(setMode(p, 'sync').positioner_angles_deg).toBeNull();
  });
  it('arcs are clamped to 1..12', () => {
    expect(setArcs(newPick('S01', 'E1', 1), 0).arcs).toBe(1);
    expect(setArcs(newPick('S01', 'E1', 1), 40).arcs).toBe(12);
  });
});

describe('validation mirrors the API', () => {
  it('accepts a good list', () => {
    expect(validatePicks([newPick('S01', 'E1', 1), newPick('S02', 'E4', 2)])).toEqual([]);
  });
  it('reports pointers', () => {
    const bad = [
      { ...newPick('X1', 'E1', 1) },
      { ...newPick('S01', 'E1', 1), positioner_mode: 'B' as const, arcs: 2, positioner_angles_deg: [0] },
      { ...newPick('S01', 'E1', 1), arc_overlap_mm: -1, start_offset_mm: -1, edges: [] },
    ];
    const ptrs = validatePicks(bad).map((e) => e.pointer);
    expect(ptrs).toContain('/seams/0/seam_id');
    expect(ptrs).toContain('/seams/1/positioner_angles_deg');
    expect(ptrs).toContain('/seams/2/arc_overlap_mm');
    expect(ptrs).toContain('/seams/2/start_offset_mm');
    expect(ptrs).toContain('/seams/2/edges');
    expect(ptrs).toContain('/seams/2/seam_id'); // duplicate S01
  });
});

describe('stored document → picks', () => {
  it('keeps only pick fields', () => {
    const picks = picksFromDoc({ seams: [ring()] });
    expect(picks).toHaveLength(1);
    expect(Object.keys(picks[0]).sort()).toEqual(
      ['arc_overlap_mm', 'arcs', 'edges', 'joint_type', 'leg_mm', 'note', 'positioner_angles_deg',
        'positioner_mode', 'range_mm', 'reversed', 'seam_id', 'sequence', 'start_offset_mm'].sort());
    expect(picks[0].positioner_mode).toBe('sync');
  });
  it('a 2.0 document without positioner_mode reads as sync', () => {
    const s = ring() as Partial<SeamDocSeam>;
    delete s.positioner_mode;
    delete s.start_offset_mm;
    expect(picksFromDoc({ seams: [s as SeamDocSeam] })[0]).toMatchObject({ positioner_mode: 'sync', start_offset_mm: 0 });
  });
});

describe('seam frames every 90° (display only)', () => {
  it('closed seam: 4 frames at quarter lengths, Z out of the material, beam = −Z', () => {
    const f = seamFrames(ring());
    expect(f.map((x) => x.label)).toEqual(['0°', '90°', '180°', '270°']);
    const L = ring().length_mm;
    f.forEach((fr, i) => {
      expect(fr.s_mm).toBeCloseTo((L * i) / 4, 6);
      expect(fr.z).toEqual([0, 0, 1]);
      expect(fr.beam).toEqual([-0, -0, -1]);
      const r = Math.hypot(fr.p[0], fr.p[1]);
      expect(r).toBeCloseTo(10, 1); // interpolated on the 1 mm polyline (chord sag ≤ 0.013 mm)
    });
    expect(f[1].p[0]).toBeCloseTo(0, 1);
    expect(f[1].p[1]).toBeCloseTo(10, 1);
  });
  it('open seam: frames at start, quarters and end', () => {
    const s = { ...ring(), closed: false };
    expect(seamFrames(s).map((x) => x.label)).toEqual(['0°', '90°', '180°', '270°', 'end']);
  });
  it('Z is the normalised bisector of n1 and n2', () => {
    const s = ring();
    s.geometry.points = s.geometry.points.map((p) => ({ ...p, n1: [0, 0, 1], n2: [1, 0, 0] }));
    const z = seamFrames(s)[0].z;
    expect(z[0]).toBeCloseTo(Math.SQRT1_2, 9);
    expect(z[2]).toBeCloseTo(Math.SQRT1_2, 9);
  });
  it('arc boundaries split the seam into equal lengths from its start', () => {
    const b = arcBoundaries(ring());
    expect(b).toHaveLength(2);
    expect(b[0].s_mm).toBe(0);
    expect(b[1].s_mm).toBeCloseTo(ring().length_mm / 2, 9);
    expect(arcBoundaries({ ...ring(), arcs: 1 })).toHaveLength(1);
  });
});
