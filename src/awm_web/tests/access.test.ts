import { describe, expect, it } from 'vitest';
import { canEditJobs } from '../src/lib/access';
import { actionBlocks } from '../src/lib/simulation';

describe('role access (CP-63: Admin ⊇ Engineer ⊇ Operator)', () => {
  it('canEditJobs: Engineer and Admin may edit, Operator may not', () => {
    expect(canEditJobs('Engineer')).toBe(true);
    expect(canEditJobs('Admin')).toBe(true);
    expect(canEditJobs('Operator')).toBe(false);
  });

  it('actionBlocks: Admin is not role-blocked from Generate / Replay, Operator is', () => {
    const sim = { path: null, path_state: 'missing', path_problems: [], replay: null } as never;
    expect(actionBlocks(sim, null, 'Operator').generate).toMatch(/Engineer or Admin/);
    // Admin passes the role gate; the only remaining block is "Plan first" (no path), not a role message.
    const admin = actionBlocks(sim, null, 'Admin');
    expect(admin.generate).not.toMatch(/Engineer or Admin/);
    expect(admin.generate).toMatch(/Plan first/);
  });
});
