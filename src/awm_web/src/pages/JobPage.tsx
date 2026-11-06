import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Vector3 } from 'three';
import { KinematicBanner, Panel, Row, RunView } from '../components/Common';
import { SeamOverlay } from '../components/SeamOverlay';
import { JOINT_COLOUR, SELECT_COLOUR, Viewer, type FocusRequest } from '../components/Viewer';
import {
  authHeaders,
  cancelRun,
  getJob,
  getSeams,
  putPlacement,
  putSeams,
  startPlan,
  viewBase,
  watchRun,
  type JobDetail,
  type Run,
  type Session,
} from '../lib/api';
import { loadFromUrl, type Model } from '../lib/load';
import { emptyPlacementForm, placementRequest, type PlacementForm } from '../lib/placement';
import type { PickTarget } from '../lib/pick';
import {
  JOINT_TYPES,
  MAX_ARCS,
  POSITIONER_MODES,
  newPick,
  nextSeamId,
  picksFromDoc,
  setArcs,
  setMode,
  validatePicks,
  type FieldError,
  type SeamDoc,
  type SeamPick,
} from '../lib/seams';
import { jointForEdge, type JointInfo } from '../lib/topology';

const fmt = (v: number, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : String(v));

/**
 * S2 seam picker (DES-001 §4.5): click an edge → its joint; seams with attributes (direction, start offset, arcs,
 * arc overlap, positioner mode, Mode B angles); save through awm_api (M2 resolves the ids); saved seams drawn with
 * their seam frame Z and beam axis every 90°; plan request with progress. Seam editing is Engineer only (CP-21).
 */
export function JobPage({ session, jobId }: { session: Session; jobId: string }) {
  const token = session.token;
  const canEdit = session.role === 'Engineer';
  const [detail, setDetail] = useState<JobDetail | null>(null);
  const [model, setModel] = useState<Model | null>(null);
  const [saved, setSaved] = useState<SeamDoc | null>(null);
  const [picks, setPicks] = useState<SeamPick[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [note, setNote] = useState<string | null>(null);
  const [fieldErrors, setFieldErrors] = useState<FieldError[]>([]);
  const [hover, setHover] = useState<PickTarget>(null);
  const [sel, setSel] = useState<PickTarget | { kind: 'joint'; id: string }>(null);
  const [activeSeam, setActiveSeam] = useState<string | null>(null);
  const [focus, setFocus] = useState<FocusRequest | null>(null);
  const [showFrames, setShowFrames] = useState(true);
  const [saving, setSaving] = useState(false);
  const [run, setRun] = useState<Run | null>(null);
  const [speed, setSpeed] = useState('');
  const [placeForm, setPlaceForm] = useState<PlacementForm>(emptyPlacementForm);
  const [placeMsg, setPlaceMsg] = useState<string | null>(null);
  const stopWatch = useRef<(() => void) | null>(null);
  useEffect(() => () => stopWatch.current?.(), []);

  const reloadDetail = useCallback(() => getJob(token, jobId).then(setDetail), [token, jobId]);

  const follow = useCallback(
    (runId: string) => {
      stopWatch.current?.();
      stopWatch.current = watchRun(
        token,
        runId,
        (r) => {
          setRun(r);
          if (r.state === 'succeeded' || r.state === 'failed' || r.state === 'cancelled') reloadDetail().catch(() => undefined);
        },
        (e) => setError(e.message),
      );
    },
    [token, reloadDetail],
  );

  useEffect(() => {
    let live = true;
    getJob(token, jobId)
      .then((d) => {
        if (!live) return;
        setDetail(d);
        if (d.active_run?.startsWith('R')) follow(d.active_run);
      })
      .catch((e: Error) => live && setError(e.message));
    loadFromUrl(viewBase(jobId), authHeaders(token))
      .then((m) => {
        if (!live) return;
        setModel(m);
        setFocus({ box: m.displayBox.clone(), key: Date.now() });
      })
      .catch((e: Error) => live && setError(`3D view model: ${e.message}`));
    getSeams(token, jobId)
      .then((doc) => {
        if (!live) return;
        setSaved(doc);
        setPicks(doc ? picksFromDoc(doc) : []);
      })
      .catch((e: Error) => live && setError(e.message));
    return () => {
      live = false;
    };
  }, [token, jobId, follow]);

  const selJoint: JointInfo | null = useMemo(() => {
    if (!model || !sel) return null;
    if (sel.kind === 'joint') return model.tidx.joints.get(sel.id) ?? null;
    if (sel.kind === 'edge') return jointForEdge(model.tidx, sel.id);
    return null;
  }, [model, sel]);

  const jointOfPick = useCallback(
    (p: SeamPick) => (model && p.edges[0] ? jointForEdge(model.tidx, p.edges[0]) : null),
    [model],
  );

  const selectedEdges = useMemo(() => {
    const s = new Set<string>();
    if (sel?.kind === 'edge') s.add(sel.id);
    if (selJoint) s.add(selJoint.edge_a).add(selJoint.edge_b);
    const act = picks.find((p) => p.seam_id === activeSeam);
    if (act) {
      act.edges.forEach((e) => s.add(e));
      const j = jointOfPick(act);
      if (j) s.add(j.edge_a).add(j.edge_b);
    }
    return s;
  }, [sel, selJoint, picks, activeSeam, jointOfPick]);

  const dirty = useMemo(
    () => JSON.stringify(picks) !== JSON.stringify(saved ? picksFromDoc(saved) : []),
    [picks, saved],
  );

  const update = (i: number, p: SeamPick) => setPicks((ps) => ps.map((q, k) => (k === i ? p : q)));

  const addSeam = () => {
    if (!selJoint || !sel) return;
    const edge = sel.kind === 'edge' ? sel.id : selJoint.edge_a;
    const id = nextSeamId(picks.map((p) => p.seam_id));
    const seq = picks.reduce((m, p) => Math.max(m, p.sequence), 0) + 1;
    if (picks.some((p) => jointOfPick(p)?.joint_edge_id === selJoint.joint_edge_id))
      setNote(`Joint ${selJoint.joint_edge_id} already has a seam; use arcs for several welds on one joint (CP-53).`);
    setPicks((ps) => [...ps, newPick(id, edge, seq)]);
    setActiveSeam(id);
  };

  const save = () => {
    const errs = validatePicks(picks);
    setFieldErrors(errs);
    if (errs.length) return;
    setSaving(true);
    setError(null);
    putSeams(token, jobId, picks)
      .then(({ doc, seamAxis }) => {
        setSaved(doc);
        setPicks(picksFromDoc(doc));
        setNote(`Saved ${doc.seams.length} seam(s). seam_axis.json: ${seamAxis || 'unknown'}.`);
        return reloadDetail();
      })
      .catch((e: Error & { problem?: { errors?: FieldError[] } }) => {
        setError(e.message);
        setFieldErrors(e.problem?.errors ?? []);
      })
      .finally(() => setSaving(false));
  };

  const speedValue = speed.trim() === '' ? undefined : Number(speed);
  const speedError = speedValue !== undefined && !(Number.isFinite(speedValue) && speedValue > 0) ? 'travel speed must be a number > 0 mm/s' : null;
  const plan = () => {
    setError(null);
    startPlan(token, jobId, undefined, speedValue)
      .then(follow)
      .catch((e: Error) => setError(e.message));
  };

  const savePlacement = () => {
    const req = placementRequest(placeForm, jobId);
    if ('errors' in req) {
      setPlaceMsg(req.errors.join('; '));
      return;
    }
    setPlaceMsg(null);
    putPlacement(token, jobId, req.body)
      .then(({ seamAxis }) => {
        setPlaceMsg(
          seamAxis === 'stale'
            ? 'Placement saved (study, Assumption). seam_axis.json is now stale: save the seams again, then re-plan.'
            : `Placement saved (study, Assumption). seam_axis.json: ${seamAxis}.`,
        );
        return reloadDetail();
      })
      .catch((e: Error) => setPlaceMsg(e.message));
  };

  const planBlock = planBlocker(detail, saved, dirty, canEdit, run);
  const size = model ? Math.max(model.displayBox.getSize(new Vector3()).length() * 0.05, 2) : 5;
  const errFor = (i: number, f: string) => fieldErrors.filter((e) => e.pointer.startsWith(`/seams/${i}/${f}`)).map((e) => e.message).join('; ');

  return (
    <div className="app">
      <header>
        <a className="btn" href="#/jobs">
          ← Jobs
        </a>
        <strong>S2 seam picker · {jobId}</strong>
        <span className="muted">{detail?.status}</span>
        <KinematicBanner />
        <span className="spacer" />
        {model && (
          <>
            <button className="btn" onClick={() => setFocus({ box: model.displayBox.clone(), key: Date.now() })}>
              Fit to view
            </button>
            <label className="check">
              <input type="checkbox" checked={showFrames} onChange={(e) => setShowFrames(e.target.checked)} /> Seam frames
            </label>
          </>
        )}
      </header>
      <main>
        <section className="view">
          {model && focus ? (
            <Viewer
              model={model}
              hover={hover}
              onHover={setHover}
              onPick={(t) => {
                setSel(t);
                setNote(t?.kind === 'edge' && model && !jointForEdge(model.tidx, t.id)
                  ? `${t.id} is not a joint edge. Goal 1 welds joints between solids only (orange edges).` : null);
              }}
              selectedEdges={selectedEdges}
              selectedFace={sel?.kind === 'face' ? sel.id : null}
              showFrames={false}
              focus={focus}
            >
              {showFrames && saved?.seams.map((s) => <SeamOverlay key={s.seam_id} seam={s} size={size} active={s.seam_id === activeSeam} />)}
            </Viewer>
          ) : (
            <div className="empty">{error ? '' : 'Loading the 3D view model …'}</div>
          )}
          {error && <div className="error">{error}</div>}
          {model && (
            <div className="overlay">
              <div>
                part frame · mm · display +Z up{model.topo.source.up_axis !== '+Z' && ` (source ${model.topo.source.up_axis})`} ·
                hover {hover ? `${hover.kind} ${hover.id}` : '—'}
              </div>
              <div>
                <span style={{ color: JOINT_COLOUR }}>orange</span> = joint edges, <span style={{ color: SELECT_COLOUR }}>cyan</span> =
                selected; saved seams green with <span style={{ color: '#5b8cff' }}>Z</span> (out of the material) and{' '}
                <span style={{ color: '#ff4fd8' }}>dashed beam axis −Z</span> every 90°. Display only — not a weld path.
              </div>
            </div>
          )}
        </section>
        <aside className="wide">
          <Panel title="Job">
            {detail ? (
              <>
                <Row k="part">
                  {detail.part_no} rev {detail.part_rev}
                </Row>
                <Row k="placement">
                  {detail.placement ? (
                    <span className="ok">
                      {detail.placement.source}
                      {detail.placement.verified ? '' : ' (not verified, simulation only)'}
                    </span>
                  ) : (
                    <span className="warn">none</span>
                  )}
                </Row>
                {detail.placement_note && <div className="warn">{detail.placement_note}</div>}
                {canEdit && (
                  <details open={!detail.placement} data-testid="placement-panel">
                    <summary>{detail.placement ? 'Change placement' : 'Give this job a placement'}</summary>
                    <div className="muted">
                      Pose of the part frame in positioner_flange (θ = 0), mm and degrees (roll-pitch-yaw, URDF convention), or a copy of another
                      job's placement. Saved as source <code>study</code>, not verified — an Assumption for simulation only.
                    </div>
                    <div className="form">
                      <label>source</label>
                      <select value={placeForm.mode} onChange={(e) => setPlaceForm({ ...placeForm, mode: e.target.value as PlacementForm['mode'] })}>
                        <option value="manual">manual pose</option>
                        <option value="copy">copy from job</option>
                      </select>
                      {placeForm.mode === 'manual' ? (
                        <>
                          <label>position mm x y z</label>
                          <span className="angles">
                            {placeForm.position_mm.map((v, i) => (
                              <input key={i} value={v} onChange={(e) => setPlaceForm({ ...placeForm, position_mm: placeForm.position_mm.map((w, k) => (k === i ? e.target.value : w)) })} />
                            ))}
                          </span>
                          <label>roll pitch yaw deg</label>
                          <span className="angles">
                            {placeForm.rpy_deg.map((v, i) => (
                              <input key={i} value={v} onChange={(e) => setPlaceForm({ ...placeForm, rpy_deg: placeForm.rpy_deg.map((w, k) => (k === i ? e.target.value : w)) })} />
                            ))}
                          </span>
                        </>
                      ) : (
                        <>
                          <label>job id</label>
                          <input value={placeForm.copy_from} placeholder="<PartNo>_<PartRev>" onChange={(e) => setPlaceForm({ ...placeForm, copy_from: e.target.value })} />
                        </>
                      )}
                    </div>
                    <button className="btn small" onClick={savePlacement}>
                      Save placement
                    </button>
                    {placeMsg && <div className="warn">{placeMsg}</div>}
                  </details>
                )}
                <Row k="seam_axis.json">
                  <span className={detail.seam_axis === 'ok' ? 'ok' : 'warn'}>{detail.seam_axis}</span>
                </Row>
                {detail.seam_axis === 'stale' && <div className="warn">Seams, placement or frames changed: save the seams again, then re-plan.</div>}
              </>
            ) : (
              <span className="muted">…</span>
            )}
          </Panel>

          <Panel title="Selection">
            {!sel && <div className="muted">Click a joint edge (orange) in the view or a joint below.</div>}
            {sel && sel.kind !== 'joint' && (
              <Row k={sel.kind}>
                <b>{sel.id}</b>
              </Row>
            )}
            {selJoint && (
              <>
                <Row k="joint">
                  <b>{selJoint.joint_edge_id}</b> · {selJoint.solid_a} ↔ {selJoint.solid_b}
                </Row>
                <Row k="edges">
                  {selJoint.edge_a}, {selJoint.edge_b}
                </Row>
                <Row k="length">
                  {fmt(selJoint.length_mm)} mm · {selJoint.closed ? 'closed' : 'open'}
                </Row>
                {canEdit && (
                  <button className="btn small" onClick={addSeam}>
                    New seam on {selJoint.joint_edge_id}
                  </button>
                )}
              </>
            )}
            {note && <div className="warn">{note}</div>}
            {model && model.topo.joints.length > 0 && (
              <table>
                <tbody>
                  {model.topo.joints.map((j) => (
                    <tr key={j.joint_edge_id} className={selJoint?.joint_edge_id === j.joint_edge_id ? 'active' : ''} onClick={() => setSel({ kind: 'joint', id: j.joint_edge_id })}>
                      <td>
                        <b>{j.joint_edge_id}</b>
                      </td>
                      <td>
                        {j.solid_a} ↔ {j.solid_b}
                      </td>
                      <td>{fmt(j.length_mm)} mm</td>
                      <td>{j.closed ? 'closed' : 'open'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
          </Panel>

          <Panel title={`Seams (${picks.length})${dirty ? ' · unsaved' : ''}`}>
            {!canEdit && <div className="muted">Role {session.role}: read only (seam editing is Engineer only, CP-21).</div>}
            {picks.map((p, i) => (
              <SeamCard
                key={i}
                pick={p}
                joint={jointOfPick(p)}
                active={p.seam_id === activeSeam}
                disabled={!canEdit || saving}
                onActivate={() => setActiveSeam(p.seam_id)}
                onChange={(q) => update(i, q)}
                onRemove={() => setPicks((ps) => ps.filter((_, k) => k !== i))}
                err={(f) => errFor(i, f)}
              />
            ))}
            {canEdit && (
              <button className="btn small" disabled={saving || !dirty} onClick={save}>
                {saving ? 'Saving …' : 'Save seams'}
              </button>
            )}
            {fieldErrors.length > 0 && <div className="err">{fieldErrors.map((e) => `${e.pointer}: ${e.message}`).join('; ')}</div>}
          </Panel>

          <Panel title="Plan (M4)">
            <KinematicBanner />
            <p className="muted">Robot + positioner, synchronised 7 axes by default (CP-59). Collision is not evaluated in Goal 1.</p>
            <div className="form">
              <label>travel speed mm/s</label>
              <input value={speed} placeholder="planner default (Assumption)" disabled={!canEdit} onChange={(e) => setSpeed(e.target.value)} />
            </div>
            {speedError && <div className="err">{speedError}</div>}
            <button className="btn small" disabled={!!planBlock || !!speedError} onClick={plan} title={planBlock ?? ''}>
              Plan all seams
            </button>
            {detail && detail.path !== 'missing' && (
              <a className="btn small" href={`#/jobs/${encodeURIComponent(jobId)}/sim`} data-testid="open-s4">
                S4 simulation →
              </a>
            )}
            {detail?.path === 'stale' && <div className="warn">path.json is stale: re-plan.</div>}
            {planBlock && <div className="muted">{planBlock}</div>}
            {run && <RunView run={run} onCancel={canEdit ? () => cancelRun(token, run.run_id).catch((e: Error) => setError(e.message)) : undefined} />}
          </Panel>
        </aside>
      </main>
    </div>
  );
}

function planBlocker(d: JobDetail | null, saved: SeamDoc | null, dirty: boolean, canEdit: boolean, run: Run | null): string | null {
  if (!canEdit) return 'Planning is requested by an Engineer.';
  if (!d) return 'Loading …';
  if (run && (run.state === 'queued' || run.state === 'running')) return 'A run is active.';
  if (!saved || saved.seams.length === 0) return 'Save at least one seam first.';
  if (dirty) return 'Save the seam changes first.';
  if (!d.placement) return 'No placement for this job: give it one in the Job panel (manual pose or copy).';
  if (d.seam_axis !== 'ok') return `seam_axis.json is ${d.seam_axis}: save the seams again.`;
  return null;
}

function SeamCard(props: {
  pick: SeamPick;
  joint: JointInfo | null;
  active: boolean;
  disabled: boolean;
  onActivate: () => void;
  onChange: (p: SeamPick) => void;
  onRemove: () => void;
  err: (field: string) => string;
}) {
  const { pick: p, joint, disabled, onChange, err } = props;
  const num = (v: string) => (v.trim() === '' ? NaN : Number(v));
  const E = ({ f }: { f: string }) => (err(f) ? <div className="err">{err(f)}</div> : null);
  return (
    <div className={`seam${props.active ? ' active' : ''}`} onClick={props.onActivate}>
      <div className="grid">
        <label>seam id</label>
        <span>
          <input value={p.seam_id} disabled={disabled} onChange={(e) => onChange({ ...p, seam_id: e.target.value.trim() })} style={{ width: 70 }} />{' '}
          <span className="muted">
            {joint ? `${joint.joint_edge_id} · ${fmt(joint.length_mm)} mm · ${joint.closed ? 'closed' : 'open'}` : `edges ${p.edges.join(', ')}`}
          </span>
          <E f="seam_id" />
        </span>
        <label>edges</label>
        <span>{p.edges.join(', ')}</span>
        <label>sequence</label>
        <input type="number" min={1} step={1} value={p.sequence} disabled={disabled} onChange={(e) => onChange({ ...p, sequence: num(e.target.value) })} />
        <label>joint type</label>
        <select value={p.joint_type} disabled={disabled} onChange={(e) => onChange({ ...p, joint_type: e.target.value as SeamPick['joint_type'] })}>
          {JOINT_TYPES.map((t) => (
            <option key={t}>{t}</option>
          ))}
        </select>
        <label>direction</label>
        <label className="check">
          <input type="checkbox" checked={p.reversed} disabled={disabled} onChange={(e) => onChange({ ...p, reversed: e.target.checked })} /> reversed
        </label>
        <label>start offset mm</label>
        <span>
          <input
            type="number"
            min={0}
            step={0.5}
            value={p.start_offset_mm}
            disabled={disabled || (joint ? !joint.closed : false)}
            onChange={(e) => onChange({ ...p, start_offset_mm: num(e.target.value) })}
          />{' '}
          <span className="muted">along the joint edge from its first vertex (closed seams)</span>
          <E f="start_offset_mm" />
        </span>
        <label>arcs</label>
        <span>
          <input type="number" min={1} max={MAX_ARCS} step={1} value={p.arcs} disabled={disabled} onChange={(e) => onChange(setArcs(p, num(e.target.value)))} />
          <E f="arcs" />
        </span>
        <label>arc overlap mm</label>
        <span>
          <input type="number" min={0} step={0.5} value={p.arc_overlap_mm} disabled={disabled} onChange={(e) => onChange({ ...p, arc_overlap_mm: num(e.target.value) })} />
          <E f="arc_overlap_mm" />
        </span>
        <label>positioner</label>
        <select value={p.positioner_mode} disabled={disabled} onChange={(e) => onChange(setMode(p, e.target.value as SeamPick['positioner_mode']))}>
          {POSITIONER_MODES.map((m) => (
            <option key={m} value={m}>
              {m === 'sync' ? 'sync (7 axes, default)' : m === 'C' ? 'C (robot at rest, positioner turns)' : 'B (fixed angle per arc)'}
            </option>
          ))}
        </select>
        {p.positioner_mode === 'B' && (
          <>
            <label>angles deg / arc</label>
            <span>
              <span className="angles">
                {(p.positioner_angles_deg ?? []).map((a, k) => (
                  <input
                    key={k}
                    type="number"
                    step={1}
                    value={a}
                    title={`arc ${k + 1}`}
                    disabled={disabled}
                    onChange={(e) => {
                      const angles = [...(p.positioner_angles_deg ?? [])];
                      angles[k] = num(e.target.value);
                      onChange({ ...p, positioner_angles_deg: angles });
                    }}
                  />
                ))}
              </span>
              <E f="positioner_angles_deg" />
            </span>
          </>
        )}
      </div>
      {!disabled && (
        <button className="btn small" onClick={props.onRemove}>
          Remove {p.seam_id}
        </button>
      )}
    </div>
  );
}
