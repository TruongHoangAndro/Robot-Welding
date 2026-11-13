/** Pure logic of the S11 jog page (CP-64): guards, readout rows, typed pose, teach-home body. No React, no network. */
import { Euler, Matrix4, Quaternion } from 'three';
import type { Problem } from './api';
import { problemMessage } from './api';
import type { JogPoseMm } from './jog';
import { matrixToPoseMm } from './jog';
import { degToRad, radToDeg } from './units';

/** 6 UR joints + positioner, order and names as D07 / playback.ts. */
export const JOG_JOINTS = [
  'shoulder_pan_joint',
  'shoulder_lift_joint',
  'elbow_joint',
  'wrist_1_joint',
  'wrist_2_joint',
  'wrist_3_joint',
  'positioner_joint',
];

/** Cartesian jog and move-to act at tcp_laser, so they need a cell TCP (head.yaml). */
export const cartesianEnabled = (hasTcp: boolean): boolean => hasTcp;

export interface ReadoutRow {
  label: 'tool0' | 'tcp_laser';
  xyz: string;
  rpy: string;
}

const f = (v: number) => (Number.isFinite(v) ? v.toFixed(2) : '—');

function row(label: ReadoutRow['label'], m: Matrix4 | null): ReadoutRow {
  if (!m) return { label, xyz: '—', rpy: '—' };
  const p = matrixToPoseMm(m);
  const e = new Euler().setFromQuaternion(new Quaternion(...(p.quat_xyzw as [number, number, number, number])), 'ZYX');
  return { label, xyz: p.xyz_mm.map(f).join(', '), rpy: [e.x, e.y, e.z].map((v) => f(radToDeg(v))).join(', ') };
}

/** Both frames side by side: tool0 and tcp_laser (null without a cell TCP). Matrices in scene units, shown in mm / deg. */
export function readoutRows(tool0: Matrix4, tcp: Matrix4 | null): ReadoutRow[] {
  return [row('tool0', tool0), row('tcp_laser', tcp)];
}

/** Typed target [x, y, z] mm and [roll, pitch, yaw] deg (yaw-pitch-roll, Z-Y-X) → API pose. */
export function poseFromInputs(v: number[]): JogPoseMm {
  const q = new Quaternion().setFromEuler(new Euler(degToRad(v[3]), degToRad(v[4]), degToRad(v[5]), 'ZYX')).normalize();
  return { xyz_mm: [v[0], v[1], v[2]], quat_xyzw: [q.x, q.y, q.z, q.w] };
}

/** PUT /cell/home body from the 7 current values (degrees). Joint angles only: frame independent. */
export function teachHomeBody(q7: number[]): { robot_joints_deg: Record<string, number>; positioner_joint_deg: number } {
  const robot: Record<string, number> = {};
  JOG_JOINTS.slice(0, 6).forEach((n, i) => (robot[n] = q7[i]));
  return { robot_joints_deg: robot, positioner_joint_deg: q7[6] };
}

/** Message for an IK / home problem; the model is not moved when this is shown. */
export const ikProblemText = (p: Problem | null | undefined, status?: number): string => problemMessage(p, status);
