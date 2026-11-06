/** CP-60 cross-check (cad_check.json, v2/cad_check): S2 banner and the gate on seam picking and Plan. */

export interface CadCheckItem {
  id: number;
  name: string;
  status: 'green' | 'yellow' | 'red' | 'skipped';
  values?: Record<string, unknown>;
  tolerance?: Record<string, number>;
  note?: string;
}
export interface CadCheck {
  status: 'green' | 'yellow' | 'red';
  checks: CadCheckItem[];
  phi_deg: number | null;
  excluded?: string[];
  unassigned_static?: string[];
}

/** Why seam picking / Plan is blocked, or null. Mirrors awm_api cad_api.cad_gate. */
export function cadBlocker(hasRoles: boolean, check: CadCheck | null, state: 'ok' | 'stale' | null): string | null {
  if (!hasRoles) return null;
  if (!check) return 'Run the CAD cross-check (roles + positioner axis) first.';
  if (state === 'stale') return 'Roles, axis, job STEP or cell model changed: re-run the cross-check.';
  if (check.status === 'red') return 'Cross-check red: fix the roles or the positioner-axis pick.';
  return null;
}

export function checkBanner(check: CadCheck, state: 'ok' | 'stale' | null): { level: string; lines: string[]; phi: string | null } {
  const lines = check.checks.map((c) => `${c.id} ${c.name}: ${c.status}${c.note ? ` — ${c.note}` : ''}`);
  return {
    level: state === 'stale' ? 'stale' : check.status,
    lines,
    phi: check.phi_deg === null || check.phi_deg === undefined ? null : check.phi_deg.toFixed(2),
  };
}
