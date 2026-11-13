import { describe, expect, it } from 'vitest';
import { Matrix4 } from 'three';
import { canEditJog } from '../src/lib/access';
import { parseRoute } from '../src/lib/route';
import { JOG_JOINTS, cartesianEnabled, poseFromInputs, readoutRows, teachHomeBody, ikProblemText } from '../src/lib/jogpage';

describe('S11 jog page logic (CP-64)', () => {
  it('canEditJog is Admin only', () => {
    expect(canEditJog('Admin')).toBe(true);
    expect(canEditJog('Engineer')).toBe(false);
    expect(canEditJog('Operator')).toBe(false);
  });

  it("route '#/jog' parses to {page:'jog'}", () => {
    expect(parseRoute('#/jog', '')).toEqual({ page: 'jog' });
  });

  it('Cartesian controls are disabled without a cell TCP, enabled with one', () => {
    expect(cartesianEnabled(false)).toBe(false);
    expect(cartesianEnabled(true)).toBe(true);
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
