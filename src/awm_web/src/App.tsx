import { useCallback, useEffect, useMemo, useState, type ReactNode } from 'react';
import { Box3, Vector3 } from 'three';
import { EDGE_COLOUR, JOINT_COLOUR, SELECT_COLOUR, Viewer, type FocusRequest } from './components/Viewer';
import { loadFromFiles, loadFromUrl, type Model } from './lib/load';
import { dataBase, niceStep, type PickTarget } from './lib/pick';
import { jointForEdge, type JointInfo } from './lib/topology';

type Selection = { kind: 'edge'; id: string } | { kind: 'face'; id: string } | { kind: 'joint'; id: string } | null;

const fmt = (v: number, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : String(v));

function jointBox(model: Model, j: JointInfo): Box3 {
  const b = new Box3();
  for (const id of [j.edge_a, j.edge_b]) for (const p of model.edgePoints.get(id) ?? []) b.expandByPoint(p);
  if (b.isEmpty()) for (const f of j.preview_frames) b.expandByPoint(new Vector3(...f.point_mm));
  return b.applyMatrix4(model.upMatrix);
}

export function App() {
  const { base, error: baseError } = useMemo(() => dataBase(window.location.search, window.location.origin), []);
  const [model, setModel] = useState<Model | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [pending, setPending] = useState<{ glb?: File; topo?: File }>({});
  const [hover, setHover] = useState<PickTarget>(null);
  const [sel, setSel] = useState<Selection>(null);
  const [showFrames, setShowFrames] = useState(false);
  const [focus, setFocus] = useState<FocusRequest | null>(null);
  const [dragging, setDragging] = useState(false);

  const accept = useCallback((m: Model) => {
    setModel(m);
    setError(null);
    setSel(null);
    setHover(null);
    setFocus({ box: m.displayBox.clone(), key: Date.now() });
  }, []);

  useEffect(() => {
    let live = true;
    if (!base) {
      setError(`${baseError}. Drop view.glb and topology.json here or use "Open files".`);
      setLoading(false);
      return;
    }
    loadFromUrl(base)
      .then((m) => live && accept(m))
      .catch((e: Error) => live && setError(`${e.message}. Drop view.glb and topology.json here or use "Open files".`))
      .finally(() => live && setLoading(false));
    return () => {
      live = false;
    };
  }, [base, baseError, accept]);

  const takeFiles = useCallback(
    (files: FileList | File[]) => {
      const next = { ...pending };
      for (const f of Array.from(files)) {
        const n = f.name.toLowerCase();
        if (n.endsWith('.glb')) next.glb = f;
        else if (n.endsWith('.json')) next.topo = f;
      }
      if (next.glb && next.topo) {
        setPending({});
        setLoading(true);
        loadFromFiles(next.glb, next.topo)
          .then(accept)
          .catch((e: Error) => setError(e.message))
          .finally(() => setLoading(false));
      } else {
        setPending(next);
        setError(`Waiting for ${next.glb ? 'topology.json' : 'view.glb'} (got ${(next.glb ?? next.topo)?.name ?? 'nothing usable'}).`);
      }
    },
    [pending, accept],
  );

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && setSel(null);
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  // Selection → highlighted edges / face
  const selJoint = useMemo(() => {
    if (!model || !sel) return null;
    if (sel.kind === 'joint') return model.tidx.joints.get(sel.id) ?? null;
    if (sel.kind === 'edge') return jointForEdge(model.tidx, sel.id);
    return null;
  }, [model, sel]);
  const selectedEdges = useMemo(() => {
    const s = new Set<string>();
    if (sel?.kind === 'edge') s.add(sel.id);
    if (selJoint) s.add(selJoint.edge_a).add(selJoint.edge_b);
    return s;
  }, [sel, selJoint]);

  const onPick = useCallback((t: PickTarget) => setSel(t), []);
  const focusJoint = (j: JointInfo) => {
    if (!model) return;
    setSel({ kind: 'joint', id: j.joint_edge_id });
    setFocus({ box: jointBox(model, j), key: Date.now() });
  };
  const link = (kind: 'edge' | 'face', id: string) => (
    <button key={id} className="idlink" onClick={() => setSel({ kind, id })}>
      {id}
    </button>
  );

  return (
    <div
      className={`app${dragging ? ' dragging' : ''}`}
      onDragOver={(e) => {
        e.preventDefault();
        setDragging(true);
      }}
      onDragLeave={() => setDragging(false)}
      onDrop={(e) => {
        e.preventDefault();
        setDragging(false);
        takeFiles(e.dataTransfer.files);
      }}
    >
      <header>
        <strong>D20 view model</strong>
        <span className="muted">read-only viewer · part frame · mm · display only, never a weld path</span>
        <span className="spacer" />
        <label className="btn">
          Open files
          <input type="file" multiple accept=".glb,.json" hidden onChange={(e) => e.target.files && takeFiles(e.target.files)} />
        </label>
        {model && (
          <>
            <button className="btn" onClick={() => setFocus({ box: model.displayBox.clone(), key: Date.now() })}>
              Fit to view
            </button>
            <label className="check">
              <input type="checkbox" checked={showFrames} onChange={(e) => setShowFrames(e.target.checked)} /> Show preview frames
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
              onPick={onPick}
              selectedEdges={selectedEdges}
              selectedFace={sel?.kind === 'face' ? sel.id : null}
              showFrames={showFrames}
              focus={focus}
            />
          ) : (
            <div className="empty">{loading ? `Loading ${base}view.glb and ${base}topology.json …` : 'No model loaded.'}</div>
          )}
          {error && <div className="error">{error}</div>}
          {model && (
            <div className="overlay">
              <div>
                Grid: {gridLabel(model)} · units mm · display +Z up
                {model.topo.source.up_axis !== '+Z' && ` (source ${model.topo.source.up_axis} rotated, display only)`}
              </div>
              {showFrames && (
                <div className="frames-note">
                  Preview frames: <span style={{ color: '#e53935' }}>X</span> tangent,{' '}
                  <span style={{ color: '#5b8cff' }}>Z</span> out of the material, dashed = beam axis −Z.{' '}
                  <b>Preview only — not a weld path.</b>
                </div>
              )}
            </div>
          )}
        </section>

        {model && (
          <aside>
            <Panel title="Solids">
              {model.topo.solids.map((s) => (
                <div key={s.solid_id} className="legend">
                  <span className="swatch" style={{ background: model.solidColour.get(s.solid_id) }} />
                  <b>{s.solid_id}</b> <span className="muted">{s.name || '(no name)'}</span>
                </div>
              ))}
              <div className="legend">
                <span className="swatch line" style={{ background: EDGE_COLOUR }} /> edge
                <span className="swatch line thick" style={{ background: JOINT_COLOUR }} /> joint edge
                <span className="swatch line thick" style={{ background: SELECT_COLOUR }} /> selected
              </div>
            </Panel>

            <Panel title={`Joints (${model.topo.joints.length})`}>
              {model.topo.joints.length === 0 && <div className="muted">No joints between separate solids.</div>}
              <table>
                <tbody>
                  {model.topo.joints.map((j) => (
                    <tr
                      key={j.joint_edge_id}
                      className={selJoint?.joint_edge_id === j.joint_edge_id ? 'active' : ''}
                      onClick={() => focusJoint(j)}
                      title="Select and focus"
                    >
                      <td>
                        <b>{j.joint_edge_id}</b>
                      </td>
                      <td>
                        {j.solid_a} ↔ {j.solid_b}
                      </td>
                      <td>{fmt(j.length_mm, 2)} mm</td>
                      <td>{j.closed ? 'closed' : 'open'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </Panel>

            <Panel title="Selection">
              {!sel && <div className="muted">Click an edge or a face. Esc clears.</div>}
              {sel?.kind === 'edge' && <EdgeDetails model={model} id={sel.id} link={link} />}
              {sel?.kind === 'face' && <FaceDetails model={model} id={sel.id} />}
              {selJoint && (
                <div className="joint">
                  <h4>Joint {selJoint.joint_edge_id}</h4>
                  <Row k="solids">
                    {selJoint.solid_a} ({model.tidx.solids.get(selJoint.solid_a)?.name || '—'}) ↔ {selJoint.solid_b} (
                    {model.tidx.solids.get(selJoint.solid_b)?.name || '—'})
                  </Row>
                  <Row k="edges">
                    {link('edge', selJoint.edge_a)} {link('edge', selJoint.edge_b)}
                  </Row>
                  <Row k="length">{fmt(selJoint.length_mm)} mm</Row>
                  <Row k="closed">{String(selJoint.closed)}</Row>
                  <Row k="preview frames">{selJoint.preview_frames.length}</Row>
                </div>
              )}
              {sel && (
                <button className="btn small" onClick={() => setSel(null)}>
                  Clear
                </button>
              )}
            </Panel>

            {model.warnings.length > 0 && (
              <Panel title="Warnings">
                {model.warnings.map((w) => (
                  <div key={w} className="warn">
                    {w}
                  </div>
                ))}
              </Panel>
            )}
          </aside>
        )}
      </main>

      <footer>
        {model ? (
          <>
            <span>
              {model.topo.solids.length} solids · {model.topo.faces.length} faces · {model.topo.edges.length} edges ·{' '}
              {model.topo.joints.length} joints
            </span>
            <span>
              source: {model.topo.source.file}
              {model.topo.source.component ? ` / ${model.topo.source.component}` : ''} (up {model.topo.source.up_axis})
            </span>
            <span>created {model.topo.created_at} by {model.topo.created_by}</span>
            <span className="muted">from {model.label}</span>
            <span className="spacer" />
            <span>hover: {hover ? `${hover.kind} ${hover.id}` : '—'}</span>
          </>
        ) : (
          <span className="muted">data base URL: {base ?? 'refused'} (set with ?data=…)</span>
        )}
      </footer>
    </div>
  );
}

function gridLabel(model: Model): string {
  const s = model.displayBox.getSize(new Vector3());
  const step = niceStep(Math.max(s.x, s.y, s.z));
  return `${step} mm cells, ${step * 10} mm sections`;
}

function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="panel">
      <h3>{title}</h3>
      {children}
    </div>
  );
}

function Row({ k, children }: { k: string; children: ReactNode }) {
  return (
    <div className="row">
      <span className="k">{k}</span>
      <span className="v">{children}</span>
    </div>
  );
}

function EdgeDetails({ model, id, link }: { model: Model; id: string; link: (k: 'edge' | 'face', id: string) => ReactNode }) {
  const e = model.tidx.edges.get(id);
  if (!e) return <div className="warn">Edge {id} is in view.glb but not in topology.json.</div>;
  return (
    <div>
      <h4>Edge {e.edge_id}</h4>
      <Row k="solid">
        {e.solid_id ?? 'shell'} {e.solid_id && model.tidx.solids.get(e.solid_id)?.name ? `(${model.tidx.solids.get(e.solid_id)!.name})` : ''}
      </Row>
      <Row k="curve type">{e.curve_type}</Row>
      <Row k="length">{fmt(e.length_mm)} mm</Row>
      <Row k="closed">{String(e.closed)}</Row>
      <Row k="adjacent faces">{e.faces.length ? e.faces.map((f) => link('face', f)) : '—'}</Row>
      <Row k="G1 neighbours">{e.tangent_neighbours.length ? e.tangent_neighbours.map((n) => link('edge', n)) : '—'}</Row>
      <Row k="joint_edge_id">{e.joint_edge_id ?? 'null'}</Row>
      <Row k="fingerprint">
        <code>{e.fingerprint}</code>
      </Row>
    </div>
  );
}

function FaceDetails({ model, id }: { model: Model; id: string }) {
  const f = model.tidx.faces.get(id);
  if (!f) return <div className="warn">Face {id} is in view.glb but not in topology.json.</div>;
  return (
    <div>
      <h4>Face {f.face_id}</h4>
      <Row k="solid">
        {f.solid_id ?? 'shell'} {f.solid_id && model.tidx.solids.get(f.solid_id)?.name ? `(${model.tidx.solids.get(f.solid_id)!.name})` : ''}
      </Row>
      <Row k="surface type">{f.surface_type}</Row>
      <Row k="area">{fmt(f.area_mm2, 2)} mm²</Row>
    </div>
  );
}
