// @vitest-environment jsdom
/**
 * Review Focus 5 (master plan G1): the q + θ the browser plays, run through the URDF chain (urdf-loader, the same
 * code the S4 view uses), reproduce the D07 TCP poses in the part frame within 0.01 mm / 0.01°.
 * Synthetic fixture always; the golden job when AWM_S4_GOLDEN_DIR names a folder with robot_description.json,
 * path.json and placement.json (written by scripts/s4_golden_files.sh).
 */
import { existsSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { sampleSegment, sampleTimeline, timelineFromPath, timelineFromProgram } from '../src/lib/playback';
import { parseRobot, tcpInPart, setJoints } from '../src/lib/robot';
import { poseErrorMmDeg } from '../src/lib/units';

/** vitest runs with the package folder as root (jsdom: import.meta.url is not a file URL). */
const FIXTURE = join(process.cwd(), 'fixtures', 's4_synthetic');
const TOL_MM = 0.01;
const TOL_DEG = 0.01;

function load(dir: string, name: string) {
  return JSON.parse(readFileSync(join(dir, name), 'utf8'));
}

function check(dir: string) {
  const urdf = load(dir, 'robot_description.json').urdf as string;
  const path = load(dir, 'path.json');
  const placement = load(dir, 'placement.json');
  const robot = parseRobot(urdf);
  const tl = timelineFromPath(path);
  let worst = { mm: 0, deg: 0, n: 0 };
  for (const seam of path.seams) {
    const timed = seam.timed[0].points;
    expect(timed.length).toBe(seam.points.length);
    seam.points.forEach((p: { pose_mm: { position_mm: number[]; quat_xyzw: number[] } }, k: number) => {
      // the played values: sample the timeline at the timed point (offset of this seam in the timeline)
      const si = tl.segments.findIndex((s) => s.seamId === seam.seam_id && s.arcIndex === seam.arc.index);
      const seg = tl.segments[si];
      // the end point of an arc is played as the end of its segment (at t1 the next arc starts)
      const last = k === seg.points.length - 1 && si < tl.segments.length - 1;
      const played = last ? sampleSegment(tl, si, seg.points[k].t) : sampleTimeline(tl, seg.points[k].t);
      setJoints(robot, tl.jointNames, played.q7);
      const err = poseErrorMmDeg(tcpInPart(robot, placement.flange_T_part), p.pose_mm);
      worst = { mm: Math.max(worst.mm, err.mm), deg: Math.max(worst.deg, err.deg), n: worst.n + 1 };
    });
  }
  return worst;
}

describe('FK of played q + θ through the URDF vs D07 TCP poses', () => {
  it('synthetic fixture (rotated base and positioner frames)', () => {
    const w = check(FIXTURE);
    expect(w.n).toBe(31);
    expect(w.mm).toBeLessThanOrEqual(TOL_MM);
    expect(w.deg).toBeLessThanOrEqual(TOL_DEG);
  });

  it('program 2.1 weld segment plays the same poses as the path', () => {
    const dir = FIXTURE;
    const robot = parseRobot(load(dir, 'robot_description.json').urdf);
    const path = load(dir, 'path.json');
    const placement = load(dir, 'placement.json');
    const tl = timelineFromProgram(load(dir, 'program_2_1.json'));
    const weld = tl.segments.find((s) => s.kind === 'weld')!;
    weld.points.forEach((pt, k) => {
      setJoints(robot, tl.jointNames, sampleTimeline(tl, pt.t).q7);
      const err = poseErrorMmDeg(tcpInPart(robot, placement.flange_T_part), path.seams[0].points[k].pose_mm);
      expect(err.mm).toBeLessThanOrEqual(TOL_MM);
      expect(err.deg).toBeLessThanOrEqual(TOL_DEG);
    });
  });

  it('a wrong θ is detected (the check is not vacuous)', () => {
    const dir = FIXTURE;
    const robot = parseRobot(load(dir, 'robot_description.json').urdf);
    const path = load(dir, 'path.json');
    const p = path.seams[0].points[5];
    setJoints(robot, timelineFromPath(path).jointNames, [...p.q_deg, p.theta_deg + 0.1]);
    expect(poseErrorMmDeg(tcpInPart(robot, load(dir, 'placement.json').flange_T_part), p.pose_mm).mm).toBeGreaterThan(0.01);
  });

  const golden = process.env.AWM_S4_GOLDEN_DIR;
  it.skipIf(!golden || !existsSync(join(golden ?? '', 'path.json')))('golden job (AWM_S4_GOLDEN_DIR)', () => {
    const w = check(golden!);
    console.log(`golden FK check: ${w.n} points, max ${w.mm.toExponential(2)} mm / ${w.deg.toExponential(2)} deg`);
    expect(w.n).toBeGreaterThan(0);
    expect(w.mm).toBeLessThanOrEqual(TOL_MM);
    expect(w.deg).toBeLessThanOrEqual(TOL_DEG);
  });
});
