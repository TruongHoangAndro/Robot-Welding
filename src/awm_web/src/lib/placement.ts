/**
 * Placement request for a job made in the browser (S2 panel, G1.6): a manual pose of the part in
 * ``positioner_flange`` (mm, roll-pitch-yaw in degrees, URDF convention) or a copy of another job's placement.
 * The server writes it as ``source: study`` with an Assumption mark; seam_axis.json then has to be rebuilt.
 */

const JOB_ID = /^[A-Za-z0-9][A-Za-z0-9._-]*_[A-Za-z0-9][A-Za-z0-9._-]*$/;
/** Same plausibility bound as the server (mm). */
export const MAX_OFFSET_MM = 5000;

export interface PlacementForm {
  mode: 'manual' | 'copy';
  position_mm: string[];
  rpy_deg: string[];
  copy_from: string;
}

export function emptyPlacementForm(): PlacementForm {
  return { mode: 'manual', position_mm: ['0', '0', '0'], rpy_deg: ['0', '0', '0'], copy_from: '' };
}

/** Request body or a list of errors (field → message). */
export function placementRequest(form: PlacementForm, jobId: string): { body: Record<string, unknown> } | { errors: string[] } {
  const errors: string[] = [];
  if (form.mode === 'copy') {
    const src = form.copy_from.trim();
    if (!JOB_ID.test(src)) errors.push('copy from: a job id <PartNo>_<PartRev>');
    else if (src === jobId) errors.push('copy from: another job is needed');
    return errors.length ? { errors } : { body: { copy_from: src } };
  }
  const num = (v: string, what: string, limit?: number) => {
    const x = v.trim() === '' ? NaN : Number(v);
    if (!Number.isFinite(x)) errors.push(`${what}: a number`);
    else if (limit !== undefined && Math.abs(x) > limit) errors.push(`${what}: |value| ≤ ${limit} mm`);
    return x;
  };
  const position_mm = form.position_mm.map((v, i) => num(v, `position ${'xyz'[i]}`, MAX_OFFSET_MM));
  const rpy_deg = form.rpy_deg.map((v, i) => num(v, ['roll', 'pitch', 'yaw'][i]));
  return errors.length ? { errors } : { body: { flange_T_part: { position_mm, rpy_deg } } };
}
