import { describe, expect, it } from 'vitest';
import { Matrix4, Quaternion, Vector3 } from 'three';
import { matrixToPoseMm, stepTarget } from '../src/lib/jog';

const rotZ90 = () => new Matrix4().makeRotationZ(Math.PI / 2).setPosition(0.1, 0.2, 0.3); // scene units = m
const pos = (m: Matrix4) => new Vector3().setFromMatrixPosition(m);

describe('jog step math (CP-64)', () => {
  it('stepTarget tool-frame +X translates along the tool axis', () => {
    const out = stepTarget(rotZ90(), 'x', 'lin', 10, 'tool'); // 10 mm
    const d = pos(out).sub(pos(rotZ90()));
    expect(d.x).toBeCloseTo(0, 9);
    expect(d.y).toBeCloseTo(0.01, 9); // tool X = world Y
    expect(d.z).toBeCloseTo(0, 9);
  });

  it('stepTarget base-frame +Z translates along world Z regardless of orientation', () => {
    const out = stepTarget(rotZ90(), 'z', 'lin', 5, 'base');
    const d = pos(out).sub(pos(rotZ90()));
    expect(d.x).toBeCloseTo(0, 9);
    expect(d.y).toBeCloseTo(0, 9);
    expect(d.z).toBeCloseTo(0.005, 9);
  });

  it('stepTarget rotation about tool Z changes orientation only, not position', () => {
    const start = rotZ90();
    const out = stepTarget(start, 'z', 'rot', 30, 'tool'); // degrees
    expect(pos(out).distanceTo(pos(start))).toBeCloseTo(0, 12);
    const q0 = new Quaternion().setFromRotationMatrix(start);
    const q1 = new Quaternion().setFromRotationMatrix(out);
    expect(q0.angleTo(q1)).toBeCloseTo(Math.PI / 6, 9);
  });

  it('matrixToPoseMm returns mm and unit quaternion', () => {
    const p = matrixToPoseMm(rotZ90());
    expect(p.xyz_mm[0]).toBeCloseTo(100, 6);
    expect(p.xyz_mm[1]).toBeCloseTo(200, 6);
    expect(p.xyz_mm[2]).toBeCloseTo(300, 6);
    expect(Math.hypot(...p.quat_xyzw)).toBeCloseTo(1, 12);
    expect(p.quat_xyzw[2]).toBeCloseTo(Math.SQRT1_2, 9); // z component of a 90 deg Z rotation
  });
});
