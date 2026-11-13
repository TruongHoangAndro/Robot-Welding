import { useEffect, useMemo, useState } from 'react';
import { Vector3 } from 'three';
import type { URDFRobot } from 'urdf-loader';
import { Panel, Row } from '../components/Common';
import { SimViewer } from '../components/SimViewer';
import { loadCellScene, type CellNode } from '../components/CellScene';
import {
  ApiError,
  authHeaders,
  cellIk,
  getCellHome,
  getRobotDescription,
  getScene,
  putCellHome,
  type Session,
} from '../lib/api';
import { canEditJog } from '../lib/access';
import { matrixToPoseMm, stepTarget, type JogAxis, type JogFrame, type JogKind } from '../lib/jog';
import { JOG_JOINTS, cartesianEnabled, ikProblemText, poseFromInputs, readoutRows, teachHomeBody } from '../lib/jogpage';
import { authMeshLoader, limitViolations, parseRobot, rootT, setJoints, TCP_LINK } from '../lib/robot';

const IDENTITY = { position_mm: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] };
const FOCUS = { target: new Vector3(0, -0.3, 0.2), offset: new Vector3(1.1, -1.5, 0.9) };
const AXES: JogAxis[] = ['x', 'y', 'z'];

/**
 * S11 manual control (CP-64, Admin): jog the saved cell's robot (joints, Cartesian at tcp_laser) and the positioner
 * in the BROWSER model, move to a typed pose, and teach the cell home. Model and analytic IK only (awm_api); nothing
 * here commands a controller, a robot or the laser (DES-001 §7.3).
 */
export function JogPage({ session }: { session: Session }) {
  const token = session.token;
  const canEdit = canEditJog(session.role);
  const [robot, setRobot] = useState<URDFRobot | null>(null);
  const [kin, setKin] = useState<URDFRobot | null>(null);
  const [cell, setCell] = useState<CellNode[]>([]);
  const [q, setQ] = useState<number[]>(JOG_JOINTS.map(() => 0));
  const [msg, setMsg] = useState<{ level: 'err' | 'ok' | 'warn'; text: string } | null>(null);
  const [noCell, setNoCell] = useState<string | null>(null);
  const [jointStep, setJointStep] = useState('5');
  const [linStep, setLinStep] = useState('10');
  const [rotStep, setRotStep] = useState('5');
  const [frame, setFrame] = useState<JogFrame>('tool');
  const [target, setTarget] = useState<string[]>(['0', '0', '0', '0', '0', '0']);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    let live = true;
    getRobotDescription(token)
      .then(async (d) => {
        if (!live) return;
        const r = parseRobot(d.urdf, authMeshLoader(authHeaders(token)), false);
        const k = parseRobot(d.urdf, undefined, false);
        setRobot(r);
        setKin(k);
        const h = await getCellHome(token).catch(() => null);
        if (live && h) setQ([...JOG_JOINTS.slice(0, 6).map((n) => Number(h.home.robot_joints_deg[n] ?? 0)), Number(h.home.positioner_joint_deg)]);
      })
      .catch((e: Error) => live && setNoCell(`No robot description: import and save the cell model in S10 first (${e.message}).`));
    getScene(token)
      .then((sc) => (sc ? loadCellScene(sc, authHeaders(token)).then(({ nodes }) => live && setCell(nodes)) : undefined))
      .catch(() => undefined);
    return () => {
      live = false;
    };
  }, [token]);

  const hasTcp = !!kin?.links[TCP_LINK];
  const cartesian = cartesianEnabled(hasTcp);
  // browser FK of the current values on the kinematics-only copy
  const fk = useMemo(() => {
    if (!kin) return null;
    setJoints(kin, JOG_JOINTS, q);
    return { tool0: rootT(kin, 'tool0'), tcp: kin.links[TCP_LINK] ? rootT(kin, TCP_LINK) : null };
  }, [kin, q]);
  const rows = fk ? readoutRows(fk.tool0, fk.tcp) : [];
  const violations = kin ? limitViolations(kin, JOG_JOINTS, q) : [];

  const accept = (next: number[]): boolean => {
    if (!kin) return false;
    const v = limitViolations(kin, JOG_JOINTS, next);
    if (v.length) {
      setMsg({ level: 'err', text: `Refused, beyond a soft limit: ${v.join('; ')}` });
      return false;
    }
    setQ(next);
    setMsg(null);
    return true;
  };
  const jogJoint = (i: number, delta: number) => {
    const next = q.slice();
    next[i] = q[i] + delta;
    accept(next);
  };
  const setJoint = (i: number, text: string) => {
    const v = Number(text);
    if (text.trim() === '' || !Number.isFinite(v)) return setMsg({ level: 'err', text: `${JOG_JOINTS[i]}: not a number` });
    const next = q.slice();
    next[i] = v;
    accept(next);
  };
  /** IK at tcp_laser, then the model moves only if IK succeeded and the result is within the limits. */
  const solve = async (pose: { xyz_mm: number[]; quat_xyzw: number[] }) => {
    setBusy(true);
    try {
      const j = await cellIk(token, pose, q.slice(0, 6));
      accept([...j, q[6]]);
    } catch (e) {
      setMsg({ level: 'err', text: e instanceof ApiError ? ikProblemText(e.problem, e.status) : (e as Error).message });
    } finally {
      setBusy(false);
    }
  };
  const jogCart = (axis: JogAxis, kind: JogKind, sign: 1 | -1) => {
    if (!fk?.tcp) return;
    const amount = Number(kind === 'lin' ? linStep : rotStep) * sign;
    if (!Number.isFinite(amount)) return setMsg({ level: 'err', text: 'step is not a number' });
    solve(matrixToPoseMm(stepTarget(fk.tcp, axis, kind, amount, frame)));
  };
  const moveTo = () => {
    const v = target.map(Number);
    if (v.some((x, i) => target[i].trim() === '' || !Number.isFinite(x))) return setMsg({ level: 'err', text: 'target: every value must be a number' });
    solve(poseFromInputs(v));
  };
  const fillTarget = () => {
    if (!fk?.tcp) return;
    const r = readoutRows(fk.tool0, fk.tcp)[1];
    setTarget([...r.xyz.split(', '), ...r.rpy.split(', ')]);
  };
  const teach = () => {
    if (!window.confirm('Teach the current joint values as the cell home? The nominal SRDF home is superseded.')) return;
    putCellHome(token, teachHomeBody(q))
      .then(() => setMsg({ level: 'ok', text: 'Home taught. The nominal SRDF home is superseded; consumers of the home (plans, programs) go stale and must be re-planned.' }))
      .catch((e: Error) =>
        setMsg({
          level: 'err',
          text: e instanceof ApiError && e.problem?.type === '/problems/no-frames' ? 'Import the cell model first (S10).' : e.message,
        }),
      );
  };

  return (
    <div className="app">
      <header>
        <a className="btn" href="#/jobs">
          ← Jobs
        </a>
        <strong>S11 manual control (jog)</strong>
        <span className="muted">model only · no controller, no robot, no laser</span>
      </header>
      <main>
        <section className="view">
          {robot ? (
            <SimViewer robot={robot} partScene={null} flangeTPart={IDENTITY} markers={[]} trail={[]} jointNames={JOG_JOINTS} q7={q} focus={FOCUS} cell={cell} />
          ) : (
            <div className="empty">{noCell ?? 'Loading the cell model …'}</div>
          )}
          {msg && (
            <div className={msg.level === 'err' ? 'error' : msg.level === 'warn' ? 'warn' : 'ok'} data-testid="jog-msg">
              {msg.text}
            </div>
          )}
        </section>
        <aside className="wide">
          <Panel title="Pose (browser model, mm / deg)">
            <table>
              <thead>
                <tr>
                  <th />
                  <th>x, y, z mm</th>
                  <th>roll, pitch, yaw °</th>
                </tr>
              </thead>
              <tbody>
                {rows.map((r) => (
                  <tr key={r.label} data-testid={`pose-${r.label}`}>
                    <td>
                      <b>{r.label}</b>
                    </td>
                    <td>{r.xyz}</td>
                    <td>{r.rpy}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            {!hasTcp && robot && <div className="warn">No TCP in this cell (head.yaml): Cartesian jog and move-to are off; joint jog and teach home still work.</div>}
            {violations.length > 0 && <div className="warn">beyond a limit: {violations.join('; ')}</div>}
          </Panel>
          <Panel title="Joint jog (6 UR joints + positioner)">
            <div className="form">
              <label>step °</label>
              <input value={jointStep} onChange={(e) => setJointStep(e.target.value)} />
            </div>
            {JOG_JOINTS.map((n, i) => (
              <div key={n} className="legend">
                <span style={{ minWidth: 130 }}>{n}</span>
                <button className="btn small" disabled={!canEdit || !kin} onClick={() => jogJoint(i, -Number(jointStep))}>
                  −
                </button>
                <input
                  key={`${n}-${q[i].toFixed(3)}`}
                  style={{ width: 80 }}
                  defaultValue={q[i].toFixed(2)}
                  disabled={!canEdit || !kin}
                  onBlur={(e) => Number(e.target.value) !== Number(q[i].toFixed(2)) && setJoint(i, e.target.value)}
                  onKeyDown={(e) => e.key === 'Enter' && (e.target as HTMLInputElement).blur()}
                />
                <button className="btn small" disabled={!canEdit || !kin} onClick={() => jogJoint(i, Number(jointStep))}>
                  +
                </button>
              </div>
            ))}
          </Panel>
          <Panel title="Cartesian jog at tcp_laser (IK via awm_api)">
            <div className="form">
              <label>frame</label>
              <select value={frame} onChange={(e) => setFrame(e.target.value as JogFrame)} disabled={!cartesian}>
                <option value="tool">Tool</option>
                <option value="base">Base</option>
              </select>
              <label>step mm</label>
              <input value={linStep} onChange={(e) => setLinStep(e.target.value)} disabled={!cartesian} />
              <label>step °</label>
              <input value={rotStep} onChange={(e) => setRotStep(e.target.value)} disabled={!cartesian} />
            </div>
            {AXES.map((a) => (
              <div key={a} className="legend">
                <b>{a.toUpperCase()}</b>
                {(['lin', 'rot'] as JogKind[]).map((k) => (
                  <span key={k} className="legend">
                    <button className="btn small" disabled={!canEdit || !cartesian || busy} onClick={() => jogCart(a, k, -1)}>
                      −{k === 'lin' ? '' : 'rot'}
                    </button>
                    <button className="btn small" disabled={!canEdit || !cartesian || busy} onClick={() => jogCart(a, k, 1)}>
                      +{k === 'lin' ? '' : 'rot'}
                    </button>
                  </span>
                ))}
              </div>
            ))}
          </Panel>
          <Panel title="Move to a pose (tcp_laser, not stored)">
            <div className="form">
              {['x mm', 'y mm', 'z mm', 'roll °', 'pitch °', 'yaw °'].map((l, i) => (
                <span key={l}>
                  <label>{l}</label>
                  <input
                    value={target[i]}
                    disabled={!cartesian}
                    onChange={(e) => setTarget(target.map((t, j) => (j === i ? e.target.value : t)))}
                  />
                </span>
              ))}
            </div>
            <div className="legend">
              <button className="btn small" disabled={!cartesian} onClick={fillTarget}>
                Fill with current
              </button>
              <button className="btn small" disabled={!canEdit || !cartesian || busy} onClick={moveTo}>
                Move
              </button>
            </div>
          </Panel>
          <Panel title="Cell home">
            <div className="muted">Stores the current joint angles (6 UR joints + positioner). Frame independent; works without a TCP.</div>
            <button className="btn small" disabled={!canEdit || !kin} onClick={teach} data-testid="teach-home">
              Teach home
            </button>
            <Row k="role">{session.role}</Row>
          </Panel>
        </aside>
      </main>
    </div>
  );
}
