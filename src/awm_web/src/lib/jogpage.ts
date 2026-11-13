/** Pure logic of the S11 jog page (CP-64): guards, readout rows, typed pose, teach-home body. No React, no network. */
import { Euler, Matrix4, Quaternion } from 'three';
import type { Problem } from './api';
import { problemMessage } from './api';
import type { JogPoseMm } from './jog';
import { matrixToPoseMm, stepTarget, type JogAxis, type JogFrame, type JogKind } from './jog';
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

/** Link the backend FK / IK are expressed in (jog.py: base_link_T_tool0). */
export const BASE_LINK = 'base_link';

/** Cartesian jog and move-to act at tcp_laser: enabled only when /cell/fk reports a taught TCP (tcp_laser not null). */
export const cartesianEnabled = (fk: { tcp_laser: unknown | null } | null | undefined): boolean => !!fk && fk.tcp_laser !== null && fk.tcp_laser !== undefined;

/** Pose of a link in the base_link frame from the two root-relative matrices (scene units): rootBase^-1 · rootLink. */
export const inBase = (rootBase: Matrix4, rootLink: Matrix4): Matrix4 => rootBase.clone().invert().multiply(rootLink);

/** API target (base_link frame, mm) of one Cartesian jog step taken from the TCP pose given as root-relative matrices. */
export function jogTargetInBase(rootBase: Matrix4, rootTcp: Matrix4, axis: JogAxis, kind: JogKind, amount: number, frame: JogFrame): JogPoseMm {
  return matrixToPoseMm(stepTarget(inBase(rootBase, rootTcp), axis, kind, amount, frame));
}

export type IkOutcome = { moved: true; q7: number[] } | { moved: false; message: string };

/**
 * Decide what an IK answer does to the model. A problem or a result beyond a soft limit never moves it.
 * ``violations`` is the limit check of the candidate (limitViolations).
 */
export function applyIk(
  current: number[],
  result: { joints_deg: number[] } | { problem: Problem | null; status?: number },
  violations: (q7: number[]) => string[],
): IkOutcome {
  if (!('joints_deg' in result)) return { moved: false, message: ikProblemText(result.problem, result.status) };
  const next = [...result.joints_deg, current[6]];
  const v = violations(next);
  if (v.length) return { moved: false, message: `Refused, beyond a soft limit: ${v.join('; ')}` };
  return { moved: true, q7: next };
}

export interface ReadoutRow {
  label: 'tool0' | 'tcp_laser';
  xyz: string;
  rpy: string;
  /** numbers behind the display strings (mm, deg); empty without a pose */
  xyzNum: number[];
  rpyNum: number[];
}

const f = (v: number) => (Number.isFinite(v) ? v.toFixed(2) : '—');

function row(label: ReadoutRow['label'], m: Matrix4 | null): ReadoutRow {
  if (!m) return { label, xyz: '—', rpy: '—', xyzNum: [], rpyNum: [] };
  const p = matrixToPoseMm(m);
  const e = new Euler().setFromQuaternion(new Quaternion(...(p.quat_xyzw as [number, number, number, number])), 'ZYX');
  const rpyNum = [e.x, e.y, e.z].map(radToDeg);
  return { label, xyz: p.xyz_mm.map(f).join(', '), rpy: rpyNum.map(f).join(', '), xyzNum: p.xyz_mm, rpyNum };
}

/** Both frames side by side (base_link frame, the caller passes inBase matrices): tool0 and tcp_laser (null without a cell TCP). Matrices in scene units, shown in mm / deg. */
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
