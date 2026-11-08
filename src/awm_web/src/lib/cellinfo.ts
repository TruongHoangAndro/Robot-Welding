/** S1 / S10 display helpers for the cell model status (CP-60, CP-61). Display only. */
import type { HeadTcp } from './api';
import { UP_AXES, type UpAxis } from './topology';

/** S1 up-axis default: the cell's up axis when a cell STEP is imported (audit m6), else +Z. */
export function defaultUpAxis(cellUp: string | null | undefined): UpAxis {
  return (UP_AXES as readonly string[]).includes(cellUp ?? '') ? (cellUp as UpAxis) : '+Z';
}

const f = (v: number, d: number) => (Number.isFinite(v) ? v.toFixed(d) : String(v));

/** S10 rows of the taught TCP: position in tool0 (mm), beam axis and its angle to tool0 +Z (deg), teach record. */
export function tcpRows(t: HeadTcp): [string, string][] {
  const rows: [string, string][] = [];
  rows.push(['method', t.method ?? 'unknown']);
  if (t.xyz_mm) rows.push(['tool0 → TCP mm', t.xyz_mm.map((v) => f(v, 2)).join(', ')]);
  if (t.beam_axis_tool0) {
    const [x, y, z] = t.beam_axis_tool0;
    const n = Math.hypot(x, y, z) || 1;
    const deg = (Math.acos(Math.max(-1, Math.min(1, z / n))) * 180) / Math.PI;
    rows.push(['beam axis (tool0)', `${[x, y, z].map((v) => f(v, 4)).join(', ')} · ${f(deg, 1)}° from tool0 +Z`]);
  }
  if (t.quat_xyzw) rows.push(['quat xyzw', t.quat_xyzw.map((v) => f(v, 4)).join(', ')]);
  if (t.component || t.edge_id) rows.push(['picked', `${t.component ?? '?'} · ${t.edge_id ?? '?'}${t.reversed ? ' · reversed' : ''}${t.mouth_radius_mm !== undefined ? ` · mouth r ${f(t.mouth_radius_mm, 2)} mm` : ''}`]);
  if (t.stickout_mm !== undefined) rows.push(['stick-out mm', f(t.stickout_mm, 2)]);
  if (t.taught_by) rows.push(['taught', `${t.taught_by} at ${t.taught_at ?? '?'}`]);
  return rows;
}

/** S1 text next to the up-axis select: where the value comes from, or why the cell default is not available. */
export function upAxisNote(cellUp: string | null, current: string, touched: boolean, error: string | null): string {
  if (error) return `cell up axis not available (${error}): default +Z`;
  if (!cellUp) return 'no cell STEP imported: default +Z';
  if (touched && current !== cellUp) return `changed (cell up axis ${cellUp})`;
  return `default from the cell (${cellUp})`;
}
