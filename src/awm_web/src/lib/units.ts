/**
 * The one unit helper of awm_web (ADR-14, rule 7): project data is mm / degrees; the S4 scene uses the unit of the
 * robot description, the **metre** (URDF, REP 103), and three.js / urdf-loader take radians. Every mm → scene and
 * deg → rad conversion in the front-end goes through this file (no scattered factors elsewhere).
 */
import { Matrix4, Quaternion, Vector3 } from 'three';

/** Scene units per millimetre (scene unit = metre). */
export const SCENE_UNITS_PER_MM = 0.001;

export const mmToScene = (mm: number): number => mm * SCENE_UNITS_PER_MM;
export const sceneToMm = (s: number): number => s / SCENE_UNITS_PER_MM;
export const degToRad = (deg: number): number => (deg * Math.PI) / 180;
export const radToDeg = (rad: number): number => (rad * 180) / Math.PI;

export interface PoseMm {
  position_mm: number[];
  quat_xyzw: number[];
}

/** ``{position_mm, quat_xyzw}`` (project data) → 4×4 in scene units. The quaternion is normalised. */
export function poseMmToMatrix(pose: PoseMm): Matrix4 {
  const [x, y, z] = pose.position_mm;
  const q = new Quaternion(...(pose.quat_xyzw as [number, number, number, number])).normalize();
  return new Matrix4().compose(new Vector3(mmToScene(x), mmToScene(y), mmToScene(z)), q, new Vector3(1, 1, 1));
}

/** 4×4 in scene units → ``{position_mm, quat_xyzw}``. */
export function matrixToPoseMm(m: Matrix4): PoseMm {
  const p = new Vector3();
  const q = new Quaternion();
  m.decompose(p, q, new Vector3());
  return { position_mm: [sceneToMm(p.x), sceneToMm(p.y), sceneToMm(p.z)], quat_xyzw: [q.x, q.y, q.z, q.w] };
}

/** Position error (mm) and rotation error (deg) between a scene-unit matrix and a pose in mm. */
export function poseErrorMmDeg(m: Matrix4, pose: PoseMm): { mm: number; deg: number } {
  const have = matrixToPoseMm(m);
  const d = have.position_mm.map((v, i) => v - pose.position_mm[i]);
  const qa = new Quaternion(...(have.quat_xyzw as [number, number, number, number]));
  const qb = new Quaternion(...(pose.quat_xyzw as [number, number, number, number])).normalize();
  return { mm: Math.hypot(d[0], d[1], d[2]), deg: radToDeg(qa.angleTo(qb)) };
}

/** Vector of mm values → scene units (positions of seam points in the part frame). */
export const vecMmToScene = (v: number[]): Vector3 => new Vector3(mmToScene(v[0]), mmToScene(v[1]), mmToScene(v[2]));
