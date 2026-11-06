import { useEffect, useState, type ReactNode } from 'react';
import { authHeaders } from '../lib/api';
import { loadFromUrl, type Model } from '../lib/load';
import type { PickTarget } from '../lib/pick';
import { Viewer, type FocusRequest } from './Viewer';

/**
 * Pick a circular edge in a D20 edge view (CP-60 positioner axis, CP-61 nozzle mouth). Only circle edges are
 * accepted; the server computes the exact centre / axis / radius from the B-Rep (pick-circle), the browser only
 * sends the edge id.
 */
export function CirclePick(props: { token: string; base: string; onEdge: (edgeId: string | null) => void; children?: ReactNode }) {
  const [model, setModel] = useState<Model | null>(null);
  const [focus, setFocus] = useState<FocusRequest | null>(null);
  const [hover, setHover] = useState<PickTarget>(null);
  const [edge, setEdge] = useState<string | null>(null);
  const [msg, setMsg] = useState<string | null>(null);
  useEffect(() => {
    let live = true;
    loadFromUrl(props.base, authHeaders(props.token))
      .then((m) => {
        if (!live) return;
        setModel(m);
        setFocus({ box: m.displayBox.clone(), key: Date.now() });
      })
      .catch((e: Error) => live && setMsg(e.message));
    return () => {
      live = false;
    };
  }, [props.base, props.token]);
  if (!model || !focus) return <div className="muted">{msg ?? 'Loading the component edges …'}</div>;
  return (
    <div>
      <div className="pickview">
        <Viewer
          model={model}
          hover={hover}
          onHover={setHover}
          onPick={(t) => {
            if (t?.kind !== 'edge') return;
            const e = model.tidx.edges.get(t.id);
            if (e?.curve_type !== 'circle') {
              setMsg(`${t.id} is a ${e?.curve_type ?? 'unknown'} edge: pick a circle (or an arc of one).`);
              return;
            }
            setMsg(null);
            setEdge(t.id);
            props.onEdge(t.id);
          }}
          selectedEdges={new Set(edge ? [edge] : [])}
          selectedFace={null}
          showFrames={false}
          focus={focus}
        >
          {props.children}
        </Viewer>
      </div>
      <div className="muted">
        {edge ? `selected circle ${edge} · ${model.tidx.edges.get(edge)?.length_mm.toFixed(2)} mm long` : 'click a circular edge'}
      </div>
      {msg && <div className="warn">{msg}</div>}
    </div>
  );
}
