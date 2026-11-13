/**
 * Role access rules in the browser (the server enforces the same in awm_api).
 *
 * CP-63 (Truong 2026-11-09): roles are hierarchical, Admin ⊇ Engineer ⊇ Operator, so an Admin may do everything an
 * Engineer may — job roles, axis pick, cross-check, seam picking / editing, process, placement, plan, program, replay,
 * release request. Admin-only screens (S10 cell model, calibration, users, release approval, CP-62 override) are gated
 * separately (role === 'Admin').
 */
export function canEditJobs(role: string): boolean {
  return role === 'Engineer' || role === 'Admin';
}
