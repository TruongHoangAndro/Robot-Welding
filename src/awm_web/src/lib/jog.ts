/**
 * Jog step math for the Admin jog page (CP-64). Pure three.js matrices, no React, no network.
 *
 * Matrices are in scene units (metre, see units.ts); step amounts and the output pose follow project data:
 * linear ``amount`` in mm, rotary ``amount`` in degrees. The output pose has the shape the API (`/cell/ik`)
 * expects: ``{xyz_mm, quat_xyzw}`` (note: units.ts ``PoseMm`` uses ``position_mm``; this file maps between them).
 */
import { Matrix4, Quaternion, Vector3 } from 'three';
import { degToRad, matrixToPoseMm as sceneMatrixToPose, mmToScene } from './units';

export type JogAxis = 'x' | 'y' | 'z';
export type JogKind = 'lin' | 'rot';
export type JogFrame = 'tool' | 'base';

/** Pose in the API shape: millimetres and a unit quaternion (x, y, z, w). */
export interface JogPoseMm {
  xyz_mm: number[];
  quat_xyzw: number[];
}

const AXES: Record<JogAxis, Vector3> = { x: new Vector3(1, 0, 0), y: new Vector3(0, 1, 0), z: new Vector3(0, 0, 1) };

/**
 * Next TCP target: ``currentTcp`` moved by one step. ``tool`` frame post-multiplies (step in the tool's own axes),
 * ``base`` frame pre-multiplies (step in the world / base axes). ``lin``: ``amount`` in mm; ``rot``: ``amount`` in degrees.
 */
export function stepTarget(currentTcp: Matrix4, axis: JogAxis, kind: JogKind, amount: number, frame: JogFrame): Matrix4 {
  const a = AXES[axis];
  const step =
    kind === 'lin'
      ? new Matrix4().makeTranslation(a.x * mmToScene(amount), a.y * mmToScene(amount), a.z * mmToScene(amount))
      : new Matrix4().makeRotationAxis(a, degToRad(amount));
  return frame === 'tool' ? currentTcp.clone().multiply(step) : step.multiply(currentTcp);
}

/** Scene-unit matrix → ``{xyz_mm, quat_xyzw}`` with a unit quaternion. */
export function matrixToPoseMm(m: Matrix4): JogPoseMm {
  const p = sceneMatrixToPose(m);
  const q = new Quaternion(...(p.quat_xyzw as [number, number, number, number])).normalize();
  return { xyz_mm: p.position_mm, quat_xyzw: [q.x, q.y, q.z, q.w] };
}
