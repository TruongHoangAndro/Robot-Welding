import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { Group } from 'three';
import { CellView, loadCellView } from '../components/CellView';
import { CirclePick } from '../components/CirclePick';
import { Panel, Row, RunView } from '../components/Common';
import { RoleTree } from '../components/RoleTree';
import {
  ApiError,
  authHeaders,
  cellFileUrl,
  cellViewBase,
  getCellRoles,
  getCellStatus,
  getCellTree,
  putCellAxis,
  putCellRoles,
  putTcp,
  saveCell,
  startComponentView,
  startHeadView,
  uploadStep,
  watchRun,
  type CellStatus,
  type Run,
  type Session,
} from '../lib/api';
import { CELL_ROLES, effectiveRoles, relPaths, type Assignment, type TreeNode } from '../lib/roles';
import { UP_AXES, type UpAxis } from '../lib/topology';

type Tab = 'roles' | 'axis' | 'tcp';

/**
 * S10 cell model (DES-001 §4.5, CP-60 / CP-61; Admin edits, others read): import the cell STEP, assign roles on the
 * component tree (robot, tool, rotary fixture, static, exclude), pick the positioner axis on a circle, teach the TCP
 * on the nozzle mouth, save the cell model. Simulation only: nothing here moves a robot or a positioner.
 */
export function CellPage({ session }: { session: Session }) {
  const token = session.token;
  const canEdit = session.role === 'Admin';
  const [status, setStatus] = useState<CellStatus | null>(null);
  const [tree, setTree] = useState<TreeNode[] | null>(null);
  const [scene, setScene] = useState<Group | null>(null);
  const [assignments, setAssignments] = useState<Assignment[]>([]);
  const [savedRoles, setSavedRoles] = useState<string>('[]');
  const [prefilled, setPrefilled] = useState(false);
  const [selected, setSelected] = useState<number | null>(null);
  const [tab, setTab] = useState<Tab>('roles');
  const [problems, setProblems] = useState<string[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [run, setRun] = useState<Run | null>(null);
  const [file, setFile] = useState<File | null>(null);
  const [upAxis, setUpAxis] = useState<UpAxis>('+Y');
  const [viewKey, setViewKey] = useState<string | null>(null);
  const [edge, setEdge] = useState<string | null>(null);
  const [reverse, setReverse] = useState(false);
  const [headComp, setHeadComp] = useState('');
  const [stickout, setStickout] = useState('0');
  const stop = useRef<(() => void) | null>(null);
  useEffect(() => () => stop.current?.(), []);

  const reload = useCallback(async () => {
    const s = await getCellStatus(token);
    setStatus(s);
    if (!s.step) return;
    const [t, r] = await Promise.all([getCellTree(token), getCellRoles(token)]);
    setTree(t);
    setAssignments(r.assignments);
    setSavedRoles(JSON.stringify(r.assignments));
    setPrefilled(r.prefilled);
    loadCellView(cellFileUrl('cell_view.glb'), authHeaders(token)).then(setScene).catch((e: Error) => setError(e.message));
  }, [token]);
  useEffect(() => {
    reload().catch((e: Error) => setError(e.message));
  }, [reload]);

  const follow = (runId: string, after?: () => void) => {
    stop.current?.();
    stop.current = watchRun(
      token,
      runId,
      (r) => {
        setRun(r);
        if (r.state === 'succeeded') after?.();
        if (r.state === 'succeeded' || r.state === 'failed') getCellStatus(token).then(setStatus).catch(() => undefined);
      },
      (e) => setError(e.message),
    );
  };
  const act = (p: Promise<unknown>) => p.catch((e: Error) => setError(e.message));

  const rel = useMemo(() => (tree ? relPaths(tree) : []), [tree]);
  const eff = useMemo(() => (tree ? effectiveRoles(tree, assignments) : []), [tree, assignments]);
  const dirty = JSON.stringify(assignments) !== savedRoles;
  const selPath = selected !== null ? rel[selected] : null;

  const upload = () => {
    if (!file) return;
    const form = new FormData();
    form.append('step', file);
    form.append('up_axis', upAxis);
    setError(null);
    act(uploadStep(token, form, () => undefined, '/cell/model').then((id) => follow(id, () => reload())));
  };
  const saveRoles = () => {
    setProblems([]);
    putCellRoles(token, assignments)
      .then(() => {
        setSavedRoles(JSON.stringify(assignments));
        setPrefilled(false);
        return getCellStatus(token).then(setStatus);
      })
      .catch((e: Error) => {
        setError(e.message);
        if (e instanceof ApiError) setProblems(e.problem?.errors?.map((x) => x.message) ?? []);
      });
  };
  const showEdges = (start: () => Promise<{ run_id: string; key: string }>) => {
    setViewKey(null);
    setEdge(null);
    act(start().then(({ run_id, key }) => follow(run_id, () => setViewKey(key))));
  };

  return (
    <div className="app">
      <header>
        <a className="btn" href="#/jobs">
          ← Jobs
        </a>
        <strong>S10 cell model</strong>
        <span className="muted">simulation only · {canEdit ? 'Admin edits' : `role ${session.role}: read only`}</span>
        <span className="spacer" />
        {(['roles', 'axis', 'tcp'] as Tab[]).map((t) => (
          <button key={t} className={`btn${tab === t ? ' active' : ''}`} onClick={() => (setTab(t), setViewKey(null), setEdge(null))}>
            {t === 'roles' ? 'Roles' : t === 'axis' ? 'Positioner axis' : 'Tool / TCP'}
          </button>
        ))}
      </header>
      <main>
        <section className="view">
          {tab === 'roles' || !viewKey ? (
            scene && status?.up_axis ? (
              <CellView scene={scene} upAxis={status.up_axis as UpAxis} roles={eff} selected={selected} onSelect={setSelected} />
            ) : (
              <div className="empty">{status?.step ? 'Loading the cell view …' : 'Import the cell STEP (panel on the right).'}</div>
            )
          ) : (
            <CirclePick key={viewKey} token={token} base={cellViewBase(viewKey)} onEdge={setEdge} />
          )}
          {error && <div className="error">{error}</div>}
        </section>
        <aside className="wide">
          <Panel title="Cell STEP">
            {status?.step ? (
              <>
                <Row k="file">{status.step.file}</Row>
                <Row k="up axis">{status.up_axis}</Row>
                <Row k="roles">{status.roles.state}{prefilled ? ' (pre-filled from the template, review)' : ''}</Row>
                <Row k="positioner axis">
                  {status.axis ? `${status.axis.edge_id} · r ${status.axis.radius_mm.toFixed(3)} mm · ${status.axis.state}` : 'not picked'}
                </Row>
                <Row k="TCP">{status.head.head_yaml ? 'head model present' : 'no head model'}</Row>
                <Row k="model">{status.model.saved ? 'saved' : 'not saved'}</Row>
                {status.stale_jobs.length > 0 && <div className="warn">Jobs to re-check / re-plan: {status.stale_jobs.join(', ')}</div>}
              </>
            ) : (
              <span className="muted">no cell STEP yet</span>
            )}
            {canEdit && (
              <details open={!status?.step}>
                <summary>{status?.step ? 'Import another cell STEP' : 'Import the cell STEP'}</summary>
                <div className="form">
                  <label>STEP</label>
                  <input type="file" accept=".step,.stp" onChange={(e) => setFile(e.target.files?.[0] ?? null)} />
                  <label>up axis of the file</label>
                  <select value={upAxis} onChange={(e) => setUpAxis(e.target.value as UpAxis)}>
                    {UP_AXES.map((a) => (
                      <option key={a}>{a}</option>
                    ))}
                  </select>
                </div>
                <button className="btn small" disabled={!file} onClick={upload}>
                  Upload and read the tree
                </button>
              </details>
            )}
          </Panel>

          {tab === 'roles' && tree && (
            <Panel title={`Roles${dirty ? ' · unsaved' : ''}`}>
              <div className="muted">Unassigned components are static obstacles. Robot geometry always comes from ur_description.</div>
              <RoleTree tree={tree} assignments={assignments} roles={CELL_ROLES} disabled={!canEdit} selected={selected} onSelect={setSelected} onChange={setAssignments} />
              {problems.length > 0 && <div className="err">{problems.join('; ')}</div>}
              {canEdit && (
                <button className="btn small" disabled={!dirty && !prefilled} onClick={saveRoles}>
                  Save roles
                </button>
              )}
            </Panel>
          )}

          {tab === 'axis' && (
            <Panel title="Positioner axis (θ = 0 = pose in the file)">
              <div className="muted">Select a rotary-fixture component in the tree (Roles tab), show its edges, click a circle of the turntable or shaft.</div>
              <Row k="component">{selPath ?? '—'}</Row>
              {canEdit && (
                <button className="btn small" disabled={!selPath} onClick={() => selPath && showEdges(() => startComponentView(token, selPath))}>
                  Show edges
                </button>
              )}
              <label className="check">
                <input type="checkbox" checked={reverse} onChange={(e) => setReverse(e.target.checked)} /> reverse axis
              </label>
              {canEdit && (
                <button className="btn small" disabled={!edge || !selPath} onClick={() => selPath && edge && act(putCellAxis(token, selPath, edge, reverse).then((id) => follow(id)))}>
                  Use {edge ?? 'circle'} as positioner axis
                </button>
              )}
              {status?.axis && (
                <div className="muted">
                  centre {status.axis.center_file_mm.map((v) => v.toFixed(2)).join(', ')} mm · axis {status.axis.axis_file.map((v) => v.toFixed(4)).join(', ')} (file frame)
                </div>
              )}
            </Panel>
          )}

          {tab === 'tcp' && (
            <Panel title="Tool / TCP (CP-61)">
              {!status?.head.step || !status.head.map ? (
                <div className="warn">Put the head STEP and its map into cell/cad (head.step, cad_map_head.yaml).</div>
              ) : (
                <>
                  <div className="muted">Component of the nozzle (instance name in the head STEP); click the circle of the nozzle mouth.</div>
                  <div className="form">
                    <label>component</label>
                    <input value={headComp} disabled={!canEdit} placeholder="instance name, e.g. Nozzle:1" onChange={(e) => setHeadComp(e.target.value)} />
                    <label>stick-out mm</label>
                    <input value={stickout} disabled={!canEdit} onChange={(e) => setStickout(e.target.value)} />
                  </div>
                  {canEdit && (
                    <button className="btn small" disabled={!headComp.trim()} onClick={() => showEdges(() => startHeadView(token, headComp.trim()))}>
                      Show edges
                    </button>
                  )}
                  <label className="check">
                    <input type="checkbox" checked={reverse} onChange={(e) => setReverse(e.target.checked)} /> reverse beam (default: out of the nozzle)
                  </label>
                  {canEdit && (
                    <button
                      className="btn small"
                      disabled={!edge || !(Number(stickout) >= 0)}
                      onClick={() => edge && act(putTcp(token, headComp.trim(), edge, reverse, Number(stickout)).then((id) => follow(id)))}
                    >
                      Teach TCP on {edge ?? 'circle'}
                    </button>
                  )}
                </>
              )}
            </Panel>
          )}

          {canEdit && status?.step && (
            <Panel title="Save">
              <div className="muted">Writes cell/model (frames, scene). Every job placed on the old cell becomes stale.</div>
              <button className="btn small" disabled={dirty || status.roles.state !== 'ok' || !status.axis} onClick={() => act(saveCell(token).then((id) => follow(id, () => reload())))}>
                Save cell model
              </button>
            </Panel>
          )}
          {run && <RunView run={run} />}
        </aside>
      </main>
    </div>
  );
}
