import { Html, Line } from '@react-three/drei';
import { useEffect, useMemo } from 'react';
import { ArrowHelper, Vector3, type Object3D } from 'three';
import { arcBoundaries, seamFrames, type SeamDocSeam } from '../lib/seams';
import { noRaycast } from './Viewer';

export const SEAM_COLOUR = '#39d353';
export const Z_COLOUR = '#1e63e9';
export const BEAM_COLOUR = '#ff4fd8';

/**
 * A saved seam (D04 geometry from M2, part frame, mm): polyline, seam frame Z and beam axis (−Z) every 90°, travel
 * direction X at the start, arc start markers. Display only — never sent back, never a weld path.
 */
export function SeamOverlay({ seam, size, active }: { seam: SeamDocSeam; size: number; active: boolean }) {
  const pts = useMemo(() => seam.geometry.points.map((p) => new Vector3(...p.p_mm)), [seam]);
  const frames = useMemo(() => {
    return seamFrames(seam).map((f, i) => {
      const p = new Vector3(...f.p);
      const z = new ArrowHelper(new Vector3(...f.z), p, size, Z_COLOUR, size * 0.25, size * 0.12);
      const x = i === 0 ? new ArrowHelper(new Vector3(...f.x), p, size * 0.8, 0xe53935, size * 0.2, size * 0.1) : null;
      for (const o of [z, x]) o?.traverse((c: Object3D) => (c.raycast = noRaycast));
      return { key: f.label, label: f.label, p, z, x, beamEnd: p.clone().addScaledVector(new Vector3(...f.beam), size * 1.4) };
    });
  }, [seam, size]);
  const arcs = useMemo(() => (seam.arcs > 1 ? arcBoundaries(seam) : []), [seam]);
  useEffect(
    () => () => {
      for (const f of frames) {
        f.z.dispose();
        f.x?.dispose();
      }
    },
    [frames],
  );
  return (
    <group>
      <Line points={pts} color={SEAM_COLOUR} lineWidth={active ? 5 : 3} raycast={noRaycast} />
      {frames.map((f) => (
        <group key={f.key}>
          <primitive object={f.z} />
          {f.x && <primitive object={f.x} />}
          <Line points={[f.p, f.beamEnd]} color={BEAM_COLOUR} lineWidth={2} dashed dashSize={size * 0.1} gapSize={size * 0.07} raycast={noRaycast} />
          <Html position={f.p} style={{ pointerEvents: 'none' }} className="frame-label">
            {seam.seam_id} {f.label}
          </Html>
        </group>
      ))}
      {arcs.map((a) => (
        <mesh key={a.index} position={a.p} raycast={noRaycast}>
          <sphereGeometry args={[size * 0.08, 12, 8]} />
          <meshBasicMaterial color="#ffe14d" />
          <Html style={{ pointerEvents: 'none' }} className="frame-label arc">
            arc {a.index}
          </Html>
        </mesh>
      ))}
    </group>
  );
}
