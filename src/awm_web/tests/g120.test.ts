import { describe, expect, it } from 'vitest';
import { replayView, type ReplayResult } from '../src/lib/replay';
import { splitScene, type SceneObject } from '../src/lib/scene';
import { actionBlocks, speedAboveMax, speedRows, staleLines, type SimulationPayload } from '../src/lib/simulation';
import { jointRows, solidLabel } from '../src/lib/topology';
import type { Run } from '../src/lib/api';

const run = (over: Partial<Run>): Run => ({
  run_id: 'R1', job_id: 'J_A', kind: 'replay', state: 'succeeded', progress: 1, started_at: '', finished_at: '',
  log_tail: [], outputs: {}, problem: null, result: null, ...over,
});
const ok: ReplayResult = { status: 'ok', final_deviation_deg: 0.0, max_tracking_error_deg: 0.058, segments: [{ segment_id: 'G001' }, { segment_id: 'G002' }] };

describe('replayView', () => {
  it('shows status, final deviation and max tracking error of a successful replay', () => {
    const v = replayView(run({ result: ok }))!;
    expect(v.status).toBe('ok');
    expect(v.finalDeviationDeg).toBe(0);
    expect(v.maxTrackingDeg).toBeCloseTo(0.058);
    expect(v.segments).toBe(2);
    expect(v.level).toBe('ok');
  });
  it('uses the mapped problem detail for a refusal', () => {
    const v = replayView(
      run({ state: 'failed', result: { status: 'refused', reason: 'x', segments: [] }, problem: { type: '/problems/replay-sim-not-running', title: 'Replay refused', status: 409, detail: 'sim_mock is not running' } }),
    )!;
    expect(v.status).toBe('refused');
    expect(v.level).toBe('err');
    expect(v.message).toContain('sim_mock is not running');
    expect(v.finalDeviationDeg).toBeNull();
  });
  it('partial: counts the replayed segments', () => {
    const v = replayView(run({ state: 'failed', result: { status: 'partial', reason: 'G002 …', segments: [{ segment_id: 'G001' }] }, problem: { type: '/problems/replay-start-state', title: 't', status: 409, detail: 'd' } }))!;
    expect(v.status).toBe('partial');
    expect(v.segments).toBe(1);
  });
  it('null for other runs or a running replay without result', () => {
    expect(replayView(run({ kind: 'plan', result: ok }))).toBeNull();
    expect(replayView(run({ state: 'running', result: null }))?.status).toBe('running');
  });
});

describe('splitScene', () => {
  const o = (id: string, parent: string): SceneObject => ({
    object_id: id, role: 'x', component: `${id}:1`, parent_frame: parent, collision: 'hull',
    pose_mm: { position_mm: [1000, 0, 0], quat_xyzw: [0, 0, 0, 1] }, mesh_url: `/api/v1/cell/meshes/scene/${id}.stl`,
  });
  it('static objects under world, rotating ones under positioner_flange, others reported', () => {
    const s = splitScene([o('a', 'world'), o('b', 'positioner_flange'), o('c', 'tool0')]);
    expect(s.world.map((x) => x.object_id)).toEqual(['a']);
    expect(s.flange.map((x) => x.object_id)).toEqual(['b']);
    expect(s.unknown.map((x) => x.object_id)).toEqual(['c']);
  });
});

const sim = (over: Partial<SimulationPayload>): SimulationPayload => ({
  job_id: 'J_A', source: 'program', program_rev: 'P001', program_state: 'ok', program: {}, path: { seams: [], rejected: [] },
  path_state: 'ok', placement: {}, seams: null, collision_checked: false, view_url: '', robot_description_url: '', notes: [],
  path_problems: [], program_problems: [],
  replay: { launch_command: 'ros2 launch …', ros_domain_id: '48', program_rev: 'P001', program_state: 'ok', problems: [], ready: true, note: '' },
  ...over,
});
const arc = (ok: boolean, max = 6.98) => ({ seam_id: 'S01', arc: { index: 1, of: 1 }, mode: 'sync', travel_speed_mm_s: 10, points: [], planning: { max_travel_speed_mm_s: max, speed_within_limits: ok } });

describe('actionBlocks', () => {
  it('both allowed for an Engineer with a current path and program', () => {
    expect(actionBlocks(sim({}), null, 'Engineer')).toEqual({ generate: null, replay: null });
  });
  it('a stale path blocks Generate and Replay with the reasons', () => {
    const b = actionBlocks(sim({ path_state: 'stale', path_problems: ['seam_axis.json: changed since this file was created'] }), null, 'Engineer');
    expect(b.generate).toContain('seam_axis.json: changed');
    expect(b.replay).toContain('path.json is stale');
  });
  it('a stale program blocks Replay only', () => {
    const b = actionBlocks(sim({ program_state: 'stale', replay: { ...sim({}).replay, program_state: 'stale', ready: false, problems: ['path.json: changed since this file was created'] } }), null, 'Engineer');
    expect(b.generate).toBeNull();
    expect(b.replay).toContain('path.json: changed');
  });
  it('an arc above its feasible maximum blocks Generate', () => {
    const s = sim({ path: { seams: [arc(false)], rejected: [] } });
    expect(speedAboveMax(speedRows(s.path))).toHaveLength(1);
    expect(actionBlocks(s, null, 'Engineer').generate).toContain('6.98');
  });
  it('no program, busy run and role', () => {
    expect(actionBlocks(sim({ program_rev: null, program_state: null, replay: { ...sim({}).replay, program_rev: null, program_state: null, ready: false } }), null, 'Engineer').replay).toContain('Generate');
    expect(actionBlocks(sim({}), run({ state: 'running' }), 'Engineer').replay).toContain('run is active');
    expect(actionBlocks(sim({}), null, 'Operator').generate).toContain('Engineer');
  });
});

describe('staleLines', () => {
  it('lists the reasons of a stale document, nothing when ok', () => {
    expect(staleLines('path.json', 'ok', [])).toEqual([]);
    expect(staleLines('program P001', 'stale', ['a', 'b'])).toEqual(['program P001 is stale: re-plan / generate again.', 'a', 'b']);
  });
});

describe('jointRows and solidLabel', () => {
  const topo = {
    solids: [
      { solid_id: 'S1', name: 's1', volume_mm3: 1, bbox_min_mm: [0, 0, 0], bbox_max_mm: [1, 1, 1], component_path: 'Asm:1 / Pipe:1' },
      { solid_id: 'S2', name: 's2', volume_mm3: 1, bbox_min_mm: [0, 0, 0], bbox_max_mm: [1, 1, 1], component_path: 'Asm:1 / Elbow:1' },
      { solid_id: 'S3', name: 's3', volume_mm3: 1, bbox_min_mm: [0, 0, 0], bbox_max_mm: [1, 1, 1], component_path: 'Jig:1 / Holder:1' },
      { solid_id: 'S4', name: 'Body 4', volume_mm3: 1, bbox_min_mm: [0, 0, 0], bbox_max_mm: [1, 1, 1] },
    ],
    joints: [
      { joint_edge_id: 'J1', solid_a: 'S1', solid_b: 'S2', edge_a: 'E1', edge_b: 'E2', length_mm: 10, closed: true, preview_frames: [] },
      { joint_edge_id: 'J2', solid_a: 'S2', solid_b: 'S3', edge_a: 'E3', edge_b: 'E4', length_mm: 5, closed: false, preview_frames: [] },
    ],
  } as never;
  it('component names (last path element), else the solid name', () => {
    expect(solidLabel(topo, 'S1')).toBe('Pipe:1');
    expect(solidLabel(topo, 'S4')).toBe('Body 4');
    expect(solidLabel(topo, 'S9')).toBe('S9');
  });
  it('hides joints with a solid that is not role part (CP-60); all joints without roles', () => {
    const roles = new Map([['S1', 'part'], ['S2', 'part'], ['S3', 'rotary_fixture']] as const);
    const r = jointRows(topo, roles as never);
    expect(r.rows.map((x) => x.joint.joint_edge_id)).toEqual(['J1']);
    expect(r.rows[0].names).toEqual(['Pipe:1', 'Elbow:1']);
    expect(r.hidden).toBe(1);
    expect(jointRows(topo, null).rows).toHaveLength(2);
  });
});

describe('replayView cancel (review I1)', () => {
  const segs = [
    { id: 'G001', durationS: 17.5 },
    { id: 'G002', durationS: 0.68 },
    { id: 'G003', durationS: 37.76 },
  ];
  const line = (id: string, kind: string, d: number) => `[INFO] [1.0] [awm_program_replay]: ${id} ${kind}: ${d.toFixed(2)} s planned, ${d.toFixed(2)} s run, max tracking error 0.1 deg (9 feedback), end deviation 0.0 deg`;
  it('a cancelled run without CLI confirmation is not "stopped": names the segment already sent and its duration', () => {
    const v = replayView(run({ state: 'cancelled', result: null, log_tail: [line('G001', 'air', 17.5)] }), segs)!;
    expect(v.status).toBe('cancelled');
    expect(v.status).not.toBe('stopped');
    expect(v.message).toContain('Cancel stops the replay CLI; the segment already sent (G002, 0.7 s) may still run on the sim_mock controller');
    expect(v.message).toContain('wait until the robot is at rest in RViz before you replay again');
  });
  it('first segment in flight when nothing was logged yet', () => {
    expect(replayView(run({ state: 'cancelled', result: null }), segs)!.message).toContain('(G001, 17.5 s)');
  });
  it('"stopped" only when the CLI confirms status cancelled', () => {
    const v = replayView(run({ state: 'cancelled', result: { status: 'cancelled', segments: [{ segment_id: 'G001' }] } }), segs)!;
    expect(v.status).toBe('stopped');
  });
});
