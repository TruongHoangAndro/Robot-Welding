import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { D07_JOINT_NAMES, sampleTimeline, timelineFromPath, timelineFromProgram, type Timeline } from '../src/lib/playback';
import { needsKinematicBanner, speedRows } from '../src/lib/simulation';

const fx = (n: string) => JSON.parse(readFileSync(new URL(`../fixtures/s4_synthetic/${n}`, import.meta.url), 'utf8'));

function programPoints(tl: Timeline) {
  return tl.segments.flatMap((s) => s.points.map((p) => ({ t: p.t, q7: p.q7, seg: s })));
}

describe('timeline from D09 2.1 (G1.5 contract: one 7-joint trajectory per segment)', () => {
  const prog = fx('program_2_1.json');
  const tl = timelineFromProgram(prog);
  it('segments in order with global times and kinds', () => {
    expect(tl.segments.map((s) => s.kind)).toEqual(['air', 'weld', 'retract']);
    expect(tl.segments[1].t0).toBeCloseTo(2.0, 12);
    expect(tl.duration).toBeCloseTo(2.0 + 12.0 + 1.5, 9);
    expect(tl.jointNames).toEqual([...prog.joint_names.robot, ...prog.joint_names.positioner]);
    expect(tl.cycleTimeS).toBe(15.5);
    expect(tl.collisionChecked).toBe(false);
    expect(tl.segments[1].positionerMode).toBe('sync');
    expect(tl.segments[1].seamId).toBe('S01');
  });
  it('played joint values equal the program values at the program times (no drift)', () => {
    for (const p of programPoints(tl)) {
      if (p.t === p.seg.t1 && p.seg !== tl.segments[tl.segments.length - 1]) continue; // next segment starts here
      const s = sampleTimeline(tl, p.t);
      expect(s.q7).toEqual(p.q7);
    }
    const weld = prog.segments[1];
    for (const pt of weld.points) expect(sampleTimeline(tl, 2.0 + pt.time_from_start_s).q7).toEqual(pt.positions_deg);
  });
  it('linear between program points, clamped outside', () => {
    const seg = tl.segments[1];
    const a = seg.points[3], b = seg.points[4];
    const s = sampleTimeline(tl, a.t + 0.25 * (b.t - a.t));
    s.q7.forEach((v, j) => expect(v).toBeCloseTo(a.q7[j] + 0.25 * (b.q7[j] - a.q7[j]), 9));
    expect(s.kind).toBe('weld');
    expect(sampleTimeline(tl, -5).q7).toEqual(tl.segments[0].points[0].q7);
    expect(sampleTimeline(tl, 1e9).q7).toEqual(tl.segments[2].points[1].q7);
  });
});

describe('timeline from D09 2.0 (robot / positioner segments with hold_deg)', () => {
  const tl = timelineFromProgram(fx('program_2_0.json'));
  it('fills the other controller from hold_deg', () => {
    expect(tl.segments[0].points[1].q7).toEqual([90, -90, 90, -90, -90, 0, 45]);
    expect(tl.segments[1].points[0].q7.slice(6)).toEqual([45]);
    expect(tl.duration).toBeCloseTo(3.0, 12);
  });
  it('refuses a segment whose positions do not fit its controller', () => {
    const bad = fx('program_2_0.json');
    bad.segments[1].points[0].positions_deg = [1, 2, 3];
    expect(() => timelineFromProgram(bad)).toThrow(/G002/);
    const bad21 = fx('program_2_1.json');
    bad21.segments[0].points[0].positions_deg = [1, 2, 3, 4, 5, 6];
    expect(() => timelineFromProgram(bad21)).toThrow(/7/);
  });
});

describe('timeline from D07 (no program yet)', () => {
  const path = fx('path.json');
  const tl = timelineFromPath(path);
  it('plays the timed segments with q and θ exactly', () => {
    expect(tl.source).toBe('path');
    expect(tl.jointNames).toEqual(D07_JOINT_NAMES);
    const pts = path.seams[0].timed[0].points;
    for (const p of pts) expect(sampleTimeline(tl, p.t_s).q7).toEqual([...p.q_deg, p.theta_deg]);
    expect(tl.cycleTimeS).toBeNull();
  });
});

describe('S4 rules', () => {
  it('banner whenever collision is not confirmed checked (CP-58)', () => {
    expect(needsKinematicBanner({ collision_checked: false })).toBe(true);
    expect(needsKinematicBanner({})).toBe(true);
    expect(needsKinematicBanner({ collision_checked: true })).toBe(false);
  });
  it('per-arc travel speed rows say "max feasible X mm/s" when over the limit', () => {
    const rows = speedRows(fx('path.json'));
    expect(rows).toHaveLength(1);
    expect(rows[0]).toMatchObject({ seam: 'S01', arc: '1/1', speed: 5, max: 4.2, ok: false });
    expect(rows[0].message).toBe('max feasible 4.20 mm/s — re-plan with a lower travel speed');
  });
});
