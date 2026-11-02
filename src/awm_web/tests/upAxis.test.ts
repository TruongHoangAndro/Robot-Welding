import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { UP_AXES, type UpAxis } from '../src/lib/topology';
import { upAxisToZ } from '../src/lib/upAxis';

const vec: Record<UpAxis, [number, number, number]> = {
  '+X': [1, 0, 0],
  '-X': [-1, 0, 0],
  '+Y': [0, 1, 0],
  '-Y': [0, -1, 0],
  '+Z': [0, 0, 1],
  '-Z': [0, 0, -1],
};

describe('upAxisToZ', () => {
  it.each(UP_AXES)('maps %s onto +Z with a proper rotation', (axis) => {
    const m = upAxisToZ(axis);
    const v = new Vector3(...vec[axis]).applyMatrix4(m);
    expect(v.x).toBeCloseTo(0, 12);
    expect(v.y).toBeCloseTo(0, 12);
    expect(v.z).toBeCloseTo(1, 12);
    expect(m.determinant()).toBeCloseTo(1, 12);
    // no scaling, no translation: lengths in mm are kept
    expect(new Vector3(3, 4, 12).applyMatrix4(m).length()).toBeCloseTo(13, 12);
    expect(new Vector3().applyMatrix4(m).length()).toBe(0);
  });
});
