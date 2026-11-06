import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';
import { degToRad, matrixToPoseMm, mmToScene, poseMmToMatrix, radToDeg, sceneToMm, SCENE_UNITS_PER_MM } from '../src/lib/units';

describe('units (the one mm / deg ↔ scene / rad helper of awm_web)', () => {
  it('scene unit is the metre of the URDF', () => {
    expect(SCENE_UNITS_PER_MM).toBe(0.001);
    expect(mmToScene(1234.5)).toBeCloseTo(1.2345, 12);
    expect(sceneToMm(mmToScene(-87.25))).toBeCloseTo(-87.25, 12);
    expect(degToRad(180)).toBe(Math.PI);
    expect(radToDeg(degToRad(-37.5))).toBeCloseTo(-37.5, 12);
  });
  it('pose mm ↔ matrix in scene units round-trips and normalises the quaternion', () => {
    const m = poseMmToMatrix({ position_mm: [10, -20, 300], quat_xyzw: [0, 0, 2, 2] });
    const p = new Vector3().setFromMatrixPosition(m);
    expect(p.toArray().map((v) => v * 1000)).toEqual([10, -20, 300].map((v) => expect.closeTo(v, 9)));
    const back = matrixToPoseMm(m);
    expect(back.position_mm[2]).toBeCloseTo(300, 9);
    const q = new Quaternion(...back.quat_xyzw);
    expect(q.angleTo(new Quaternion(0, 0, Math.SQRT1_2, Math.SQRT1_2))).toBeLessThan(1e-12);
  });
});
