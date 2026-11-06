import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Vector3, type Object3D } from 'three';
import type { URDFRobot } from 'urdf-loader';
import { Panel, Row, RunView } from '../components/Common';
import { SimViewer, type TrailPoint } from '../components/SimViewer';
import {
  authHeaders,
  cancelRun,
  getJob,
  getRobotDescription,
  getSimulation,
  startPlan,
  startProgram,
  viewBase,
  watchRun,
  type JobDetail,
  type Run,
  type Session,
} from '../lib/api';
import { loadFromUrl } from '../lib/load';
import { sampleTimeline, timelineFromPath, timelineFromProgram, type Timeline } from '../lib/playback';
import { authMeshLoader, limitViolations, parseRobot, rootT, setJoints } from '../lib/robot';
import { poseMmToMatrix, vecMmToScene } from '../lib/units';
import { KINEMATIC_BANNER, needsKinematicBanner, seamMarkers, speedRows, STATUS_COLOURS, type SimulationPayload } from '../lib/simulation';

const fmt = (v: number | null | undefined, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—');
const RATES = [0.25, 0.5, 1, 2, 4, 8];

function buildTimeline(sim: SimulationPayload): Timeline {
  return sim.source === 'program' && sim.program ? timelineFromProgram(sim.program) : timelineFromPath(sim.path);
}

/** TCP trail: FK of every timeline point through the URDF (kinematics-only copy of the robot), world frame. */
function buildTrail(kin: URDFRobot, tl: Timeline): TrailPoint[] {
  const out: TrailPoint[] = [];
  for (const seg of tl.segments)
    for (const p of seg.points) {
      setJoints(kin, tl.jointNames, p.q7);
      out.push({ p: new Vector3().setFromMatrixPosition(rootT(kin, 'tcp_laser')), weld: seg.kind.includes('weld') });
    }
  return out;
}

/**
 * S4-lite (DES-001 §4.5 S4, G1.6): playback of the stored program (D09) — or of the planned path (D07) before a
 * program exists — with the robot (URDF with the H-CAD head), the rotary positioner (θ) and the part on its flange.
 * Display only: nothing here commands the robot, the positioner, the PLC or the laser (§7.3).
 */
export function SimPage({ session, jobId }: { session: Session; jobId: string }) {
  const token = session.token;
  const canRun = session.role === 'Engineer';
  const [sim, setSim] = useState<SimulationPayload | null | undefined>(undefined);
  const [detail, setDetail] = useState<JobDetail | null>(null);
  const [robot, setRobot] = useState<URDFRobot | null>(null);
  const [kin, setKin] = useState<URDFRobot | null>(null);
  const [part, setPart] = useState<Object3D | null>(null);
  const [partCentreMm, setPartCentreMm] = useState<number[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [rate, setRate] = useState(1);
  const [speed, setSpeed] = useState('');
  const [run, setRun] = useState<Run | null>(null);
  const stopWatch = useRef<(() => void) | null>(null);
  useEffect(() => () => stopWatch.current?.(), []);

  const reload = useCallback(() => {
    getJob(token, jobId).then(setDetail).catch((e: Error) => setError(e.message));
    return getSimulation(token, jobId)
      .then((s) => {
        setSim(s);
        const v = s?.path?.seams?.[0]?.travel_speed_mm_s;
        if (typeof v === 'number') setSpeed(String(v));
      })
      .catch((e: Error) => setError(e.message));
  }, [token, jobId]);

  useEffect(() => {
    let live = true;
    reload();
    getRobotDescription(token)
      .then((d) => {
        if (!live) return;
        setRobot(parseRobot(d.urdf, authMeshLoader(authHeaders(token))));
        setKin(parseRobot(d.urdf));
      })
      .catch((e: Error) => live && setError(`robot description: ${e.message}`));
    loadFromUrl(viewBase(jobId), authHeaders(token))
      .then((m) => {
        if (!live) return;
        setPart(m.scene);
        // bounding-box centre of the solids in the part frame (mm, topology.json), for the camera only
        const lo = [0, 1, 2].map((i) => Math.min(...m.topo.solids.map((x) => x.bbox_min_mm[i])));
        const hi = [0, 1, 2].map((i) => Math.max(...m.topo.solids.map((x) => x.bbox_max_mm[i])));
        setPartCentreMm(lo.map((v, i) => (v + hi[i]) / 2));
      })
      .catch((e: Error) => live && setError(`part view model: ${e.message}`));
    return () => {
      live = false;
    };
  }, [token, jobId, reload]);

  const [timeline, timelineError] = useMemo((): [Timeline | null, string | null] => {
    if (!sim) return [null, null];
    try {
      return [buildTimeline(sim), null];
    } catch (e) {
      return [null, `playback data refused: ${(e as Error).message}`];
    }
  }, [sim]);
  const trail = useMemo(() => (kin && timeline ? buildTrail(kin, timeline) : []), [kin, timeline]);
  const markers = useMemo(() => (sim ? seamMarkers(sim.path, sim.seams) : []), [sim]);
  const [focusOn, setFocusOn] = useState<'part' | 'cell'>('part');
  const focus = useMemo(() => {
    if (!kin || !sim) return { target: new Vector3(0, -0.3, 0.2), offset: new Vector3(1.1, -1.5, 0.9) };
    setJoints(kin, ['positioner_joint'], [0]);
    const flange = rootT(kin, 'positioner_flange');
    if (focusOn === 'cell')
      return { target: new Vector3().setFromMatrixPosition(flange).multiplyScalar(0.6), offset: new Vector3(1.1, -1.5, 0.9) };
    // centre of the part on the flange at θ = 0 (display only)
    const part = flange.clone().multiply(poseMmToMatrix(sim.placement.flange_T_part));
    const c = partCentreMm ? vecMmToScene(partCentreMm).applyMatrix4(part) : new Vector3().setFromMatrixPosition(part);
    return { target: c, offset: new Vector3(0.38, -0.5, 0.3) };
  }, [kin, sim, focusOn, partCentreMm]);

  // play loop: real time × rate; stops at the end
  useEffect(() => {
    if (!playing || !timeline) return;
    let raf = 0;
    let last = performance.now();
    const step = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;
      setT((v) => {
        const nv = Math.min(timeline.duration, v + dt * rate);
        if (nv >= timeline.duration) setPlaying(false);
        return nv;
      });
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, [playing, rate, timeline]);

  const sample = useMemo(() => (timeline ? sampleTimeline(timeline, t) : null), [timeline, t]);
  const violations = useMemo(() => (robot && sample && timeline ? limitViolations(robot, timeline.jointNames, sample.q7) : []), [robot, sample, timeline]);

  const follow = (runId: string) => {
    stopWatch.current?.();
    stopWatch.current = watchRun(
      token,
      runId,
      (r) => {
        setRun(r);
        if (r.state === 'succeeded') {
          setT(0);
          reload();
        }
      },
      (e) => setError(e.message),
    );
  };
  const speedValue = speed.trim() === '' ? undefined : Number(speed);
  const speedError = speedValue !== undefined && !(Number.isFinite(speedValue) && speedValue > 0) ? 'travel speed must be a number > 0 mm/s' : null;
  const busy = run && (run.state === 'queued' || run.state === 'running');
  const plan = () => {
    setError(null);
    startPlan(token, jobId, undefined, speedValue).then(follow).catch((e: Error) => setError(e.message));
  };
  const program = () => {
    setError(null);
    startProgram(token, jobId).then(follow).catch((e: Error) => setError(e.message));
  };

  const rows = speedRows(sim?.path ?? null);
  const showBanner = sim ? needsKinematicBanner(sim) : true;

  return (
    <div className="app">
      <header>
        <a className="btn" href={`#/jobs/${encodeURIComponent(jobId)}`}>
          ← S2 seams
        </a>
        <strong>S4 simulation · {jobId}</strong>
        {showBanner && (
          <div className="banner" role="alert" data-testid="kinematic-banner">
            {KINEMATIC_BANNER}
          </div>
        )}
        <span className="spacer" />
        <label className="check">
          focus
          <select value={focusOn} data-testid="focus" onChange={(e) => setFocusOn(e.target.value as 'part' | 'cell')}>
            <option value="part">part</option>
            <option value="cell">cell</option>
          </select>
        </label>
        {timeline && (
          <span className="muted">
            playing {timeline.source === 'program' ? `program ${timeline.programRev}` : 'planned path (no program yet)'}
          </span>
        )}
      </header>
      <main>
        <section className="view">
          {robot && sim && sample && timeline ? (
            <SimViewer
              robot={robot}
              partScene={part}
              flangeTPart={sim.placement.flange_T_part}
              markers={markers}
              trail={trail}
              jointNames={timeline.jointNames}
              q7={sample.q7}
              focus={focus}
            />
          ) : (
            <div className="empty">{sim === null ? 'Nothing planned yet: save the seams in S2 and plan.' : error ? '' : 'Loading robot, part and playback data …'}</div>
          )}
          {(error || timelineError) && <div className="error">{[error, timelineError].filter(Boolean).join('\n')}</div>}
          {sample && (
            <div className="overlay" data-testid="sim-overlay">
              t {fmt(sample.t)} / {fmt(timeline?.duration)} s · segment <b>{sample.kind}</b>
              {sample.seamId ? ` · ${sample.seamId}` : ''}
              {sample.positionerMode ? ` · mode ${sample.positionerMode}` : ''} · θ {fmt(sample.q7[6], 2)}° · world frame, Z up · display only
            </div>
          )}
        </section>
        <aside className="wide">
          <Panel title="Playback">
            {timeline ? (
              <>
                <div className="legend">
                  <button className="btn" data-testid="play" onClick={() => (t >= timeline.duration ? (setT(0), setPlaying(true)) : setPlaying(!playing))}>
                    {playing ? 'Pause' : 'Play'}
                  </button>
                  <select value={rate} onChange={(e) => setRate(Number(e.target.value))} title="playback rate">
                    {RATES.map((r) => (
                      <option key={r} value={r}>
                        {r}×
                      </option>
                    ))}
                  </select>
                  <span className="muted">
                    {fmt(t)} / {fmt(timeline.duration)} s
                  </span>
                </div>
                <input
                  type="range"
                  data-testid="scrubber"
                  min={0}
                  max={timeline.duration}
                  step={Math.max(timeline.duration / 2000, 0.001)}
                  value={t}
                  style={{ width: '100%' }}
                  onChange={(e) => {
                    setPlaying(false);
                    setT(Number(e.target.value));
                  }}
                />
                <Row k="cycle time">{timeline.cycleTimeS !== null ? `${fmt(timeline.cycleTimeS, 1)} s (program)` : `${fmt(timeline.duration, 1)} s (weld segments of the path only)`}</Row>
                <Row k="segment">
                  <span data-testid="segment-kind">{sample?.kind}</span> {sample?.seamId ?? ''} {sample?.positionerMode ? `(${sample.positionerMode})` : ''}
                </Row>
                <Row k="θ positioner">{fmt(sample?.q7[6], 3)}°</Row>
                <Row k="q robot">{sample?.q7.slice(0, 6).map((v) => fmt(v, 1)).join(', ')}°</Row>
                {violations.length > 0 && <div className="warn">beyond a URDF limit: {violations.join('; ')}</div>}
                <div className="muted">Joint values are the stored {timeline.source} values, linear between its points (no re-planning in the browser).</div>
              </>
            ) : (
              <span className="muted">—</span>
            )}
          </Panel>
          <Panel title="Seam points (D07 status)">
            <div className="legend">
              {Object.entries(STATUS_COLOURS).map(([k, c]) => (
                <span key={k} className="legend">
                  <span className="swatch" style={{ background: c }} /> {k}
                </span>
              ))}
            </div>
            <div className="legend">
              <span className="swatch line thick" style={{ background: '#ff8c1a' }} /> TCP trail, weld
              <span className="swatch line thick" style={{ background: '#8f99a6' }} /> other moves
            </div>
            {sim?.path?.rejected?.length ? (
              <div className="warn">
                rejected: {sim.path.rejected.map((r: { seam_id: string; arc: { index: number; of: number }; reason: string }) => `${r.seam_id} arc ${r.arc.index}/${r.arc.of} (${r.reason})`).join('; ')}
              </div>
            ) : null}
          </Panel>
          <Panel title="Travel speed per arc (CP-59)">
            <table>
              <tbody>
                {rows.map((r) => (
                  <tr key={`${r.seam}-${r.arc}`}>
                    <td>
                      <b>{r.seam}</b> {r.arc} {r.mode}
                    </td>
                    <td>{fmt(r.speed)} mm/s</td>
                    <td className={r.ok === false ? 'warn' : r.ok ? 'ok' : 'muted'} data-testid="speed-msg">
                      {r.message}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
            <div className="muted">Max feasible = largest travel speed relative to the part that keeps every joint and the positioner within its speed limit. The software never slows down silently.</div>
          </Panel>
          <Panel title="Plan and program (Engineer)">
            <div className="form">
              <label>travel speed mm/s</label>
              <input data-testid="speed" value={speed} placeholder="planner default (Assumption)" disabled={!canRun} onChange={(e) => setSpeed(e.target.value)} />
            </div>
            {speedError && <div className="err">{speedError}</div>}
            <div className="legend">
              <button className="btn small" disabled={!canRun || !!busy || !!speedError || detail?.seam_axis !== 'ok'} onClick={plan} data-testid="replan">
                Re-plan
              </button>
              <button className="btn small" disabled={!canRun || !!busy || !sim?.path || sim.path_state !== 'ok'} onClick={program} data-testid="generate">
                Generate program
              </button>
            </div>
            {!canRun && <div className="muted">Role {session.role}: read only (runs are started by an Engineer).</div>}
            {detail && detail.seam_axis !== 'ok' && <div className="warn">seam_axis.json is {detail.seam_axis}: save the seams again in S2, then re-plan.</div>}
            {sim?.notes.map((n) => (
              <div key={n} className="warn">
                {n}
              </div>
            ))}
            {run && <RunView run={run} onCancel={canRun ? () => cancelRun(token, run.run_id).catch((e: Error) => setError(e.message)) : undefined} />}
          </Panel>
        </aside>
      </main>
    </div>
  );
}
