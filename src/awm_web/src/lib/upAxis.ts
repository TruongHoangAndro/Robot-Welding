import { Matrix4 } from 'three';
import type { UpAxis } from './topology';

/**
 * Proper rotation (det +1) that maps the source up axis onto +Z. Display only: the data stays in the part frame
 * (contract rule 2); the viewer applies this matrix to the group that holds the model.
 */
export function upAxisToZ(axis: UpAxis): Matrix4 {
  const m = new Matrix4();
  const h = Math.PI / 2;
  switch (axis) {
    case '+Z':
      return m.identity();
    case '-Z':
      return m.makeRotationX(Math.PI); // -Z -> +Z
    case '+Y':
      return m.makeRotationX(h); // +Y -> +Z
    case '-Y':
      return m.makeRotationX(-h); // -Y -> +Z
    case '+X':
      return m.makeRotationY(-h); // +X -> +Z
    case '-X':
      return m.makeRotationY(h); // -X -> +Z
  }
}
