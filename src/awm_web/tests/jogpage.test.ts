import { describe, expect, it } from 'vitest';
import { Matrix4 } from 'three';
import { canEditJog } from '../src/lib/access';
import { parseRoute } from '../src/lib/route';
import { JOG_JOINTS, applyIk, cartesianEnabled, inBase, jogTargetInBase, poseFromInputs, readoutRows, teachHomeBody, ikProblemText } from '../src/lib/jogpage';

describe('S11 jog page logic (CP-64)', () => {
  it('canEditJog is Admin only', () => {
    expect(canEditJog('Admin')).toBe(true);
    expect(canEditJog('Engineer')).toBe(false);
    expect(canEditJog('Operator')).toBe(false);
  });

  it("route '#/jog' parses to {page:'jog'}", () => {
    expect(parseRoute('#/jog', '')).toEqual({ page: 'jog' });
  });

  it('Cartesian enabled follows /cell/fk: tcp_laser null = off, a pose = on, no answer = off', () => {
    const pose = { xyz_mm: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] };
    expect(cartesianEnabled({ tcp_laser: null })).toBe(false);
    expect(cartesianEnabled({ tcp_laser: pose })).toBe(true);
    expect(cartesianEnabled(null)).toBe(false);
  });

  it('non-identity base: pose and jog target sent to /cell/ik are in base_link, not world', () => {
    const rootBase = new Matrix4().makeTranslation(0.5, 0, 0); // base_link 500 mm from world
    const rootTcp = new Matrix4().makeTranslation(0.6, 0, 0.3);
    expect(inBase(rootBase, rootTcp).elements[12]).toBeCloseTo(0.1, 12);
    const t = jogTargetInBase(rootBase, rootTcp, 'x', 'lin', 10, 'base');
    expect(t.xyz_mm[0]).toBeCloseTo(110, 6); // 100 mm in base_link + 10; world would give 610
    expect(t.xyz_mm[1]).toBeCloseTo(0, 6);
    expect(t.xyz_mm[2]).toBeCloseTo(300, 6);
  });

  it('applyIk: a problem leaves the model unchanged, a success within limits moves, a limit violation is refused', () => {
    const cur = [0, 0, 0, 0, 0, 0, 12];
    const none = () => [];
    const bad = applyIk(cur, { problem: { type: '/problems/ik-unreachable', title: 'Target out of reach', status: 422 }, status: 422 }, none);
    expect(bad).toEqual({ moved: false, message: expect.stringMatching(/out of reach/) });
    const ok = applyIk(cur, { joints_deg: [1, 2, 3, 4, 5, 6] }, none);
    expect(ok).toEqual({ moved: true, q7: [1, 2, 3, 4, 5, 6, 12] });
    const lim = applyIk(cur, { joints_deg: [1, 2, 3, 4, 5, 400] }, () => ['wrist_3_joint 400.00° outside']);
    expect(lim.moved).toBe(false);
  });

  it('readout shows both tool0 and tcp_laser; tcp_laser row is a dash without a TCP', () => {
    const t = new Matrix4().makeTranslation(0.1, 0.2, 0.3); // scene units (m)
    const both = readoutRows(t, t);
    expect(both.map((r) => r.label)).toEqual(['tool0', 'tcp_laser']);
    expect(both[0].xyz).toBe('100.00, 200.00, 300.00');
    expect(readoutRows(t, null)[1].xyz).toBe('—');
  });

  it('typed pose becomes the API shape, mm and unit quaternion', () => {
    const p = poseFromInputs([10, 20, 30, 0, 0, 90]);
    expect(p.xyz_mm).toEqual([10, 20, 30]);
    expect(p.quat_xyzw[2]).toBeCloseTo(Math.SQRT1_2, 9);
    expect(p.quat_xyzw[3]).toBeCloseTo(Math.SQRT1_2, 9);
  });

  it('teach home body: 6 UR joints by name + positioner, joint order matches D07', () => {
    expect(JOG_JOINTS).toHaveLength(7);
    const b = teachHomeBody([1, 2, 3, 4, 5, 6, 7]);
    expect(Object.keys(b.robot_joints_deg)).toEqual(JOG_JOINTS.slice(0, 6));
    expect(b.robot_joints_deg.wrist_3_joint).toBe(6);
    expect(b.positioner_joint_deg).toBe(7);
  });

  it('ik problem text names the problem', () => {
    expect(ikProblemText({ type: '/problems/ik-unreachable', title: 'Unreachable', status: 422 })).toMatch(/Unreachable/);
  });
});
