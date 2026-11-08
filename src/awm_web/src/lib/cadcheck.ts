/**
 * CP-60 cross-check (cad_check.json, v2/cad_check): S2 banner and the gate on seam picking and Plan; CP-62 Admin
 * override of check 4 for kinematic-only planning (cad_check_override.json, v2/cad_check_override).
 */

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

/** CP-62 override record (v2/cad_check_override; header fields omitted). */
export interface CadOverride {
  by: string;
  created_at: string;
  reason: string;
  findings: string[];
  check_id: 4;
  scope: 'kinematic_planning';
  cad_check_sha256: string;
}
/** GET /jobs/{id}/check/override: valid false = void (check changed) or not matching the schema. */
export interface OverrideState {
  override: CadOverride;
  valid: boolean;
  problem: string | null;
}

export interface CadGate {
  /** Why seam picking / Plan is blocked, or null. */
  block: string | null;
  /** Banner text of a valid CP-62 override, or null. */
  overridden: string | null;
  /** An Admin may record (or re-record) the CP-62 override: check 4 is the only red check and the check is current. */
  canOverride: boolean;
}

const OVERRIDE_CHECK = 4;

/** Names of the static objects hit by the swept rotating set (check 4 values.hits[].static). */
export function check4Statics(check: CadCheck): string[] {
  const c4 = check.checks.find((c) => c.id === OVERRIDE_CHECK);
  const hits = (c4?.values?.hits ?? []) as { static?: string }[];
  return [...new Set(hits.map((h) => h?.static).filter((s): s is string => !!s))].sort();
}

export function overrideBanner(o: CadOverride): string {
  return `Overridden for kinematic planning by ${o.by} at ${o.created_at}: ${o.reason}`;
}

/** S2 gate. Mirrors awm_api cad_api.cad_gate (the server decides; this only explains). */
export function cadGate(hasRoles: boolean, check: CadCheck | null, state: 'ok' | 'stale' | null, override: OverrideState | null): CadGate {
  const none: CadGate = { block: null, overridden: null, canOverride: false };
  if (!hasRoles) return none;
  if (!check) return { ...none, block: 'Run the CAD cross-check (roles + positioner axis) first.' };
  if (state === 'stale') return { ...none, block: 'Roles, axis, job STEP or cell model changed: re-run the cross-check.' };
  if (check.status !== 'red') return none;
  const reds = check.checks.filter((c) => c.status === 'red');
  const others = reds.filter((c) => c.id !== OVERRIDE_CHECK);
  if (!reds.length || others.length) {
    const list = others.map((c) => `check ${c.id}: ${c.note ?? c.name}`).join('; ');
    return { ...none, block: `Cross-check red${list ? ` (${list})` : ''}: fix the roles or the positioner-axis pick and re-run the cross-check.` };
  }
  const statics = check4Statics(check);
  const cause =
    `Check 4 red: static object(s) inside the volume swept by the rotating set${statics.length ? `: ${statics.join(', ')}` : ''}. ` +
    'The cause is in the CAD (OI-23): separate these objects in the cell CAD, or correct their roles (S10 cell roles or the job roles here).';
  if (!override) return { block: `${cause} An Admin may record a CP-62 override for kinematic planning (collision off).`, overridden: null, canOverride: true };
  if (!override.valid)
    return {
      block: `The CP-62 override recorded by ${override.override.by} is void: ${override.problem ?? 'record not valid'}. ${cause}`,
      overridden: null,
      canOverride: true,
    };
  return { block: null, overridden: overrideBanner(override.override), canOverride: false };
}

/** Why seam picking / Plan is blocked, or null (kept for callers without the override). */
export function cadBlocker(hasRoles: boolean, check: CadCheck | null, state: 'ok' | 'stale' | null, override: OverrideState | null = null): string | null {
  return cadGate(hasRoles, check, state, override).block;
}

export function checkBanner(check: CadCheck, state: 'ok' | 'stale' | null): { level: string; lines: string[]; phi: string | null } {
  const lines = check.checks.map((c) => `${c.id} ${c.name}: ${c.status}${c.note ? ` — ${c.note}` : ''}`);
  return {
    level: state === 'stale' ? 'stale' : check.status,
    lines,
    phi: check.phi_deg === null || check.phi_deg === undefined ? null : check.phi_deg.toFixed(2),
  };
}
