import { GizmoHelper, GizmoViewport, Grid, Line, OrbitControls } from '@react-three/drei';
import { Canvas, useThree, type ThreeEvent } from '@react-three/fiber';
import { memo, useEffect, useMemo } from 'react';
import { ArrowHelper, Box3, Color, PerspectiveCamera, Sphere, Vector3, type Object3D } from 'three';
import type { Model } from '../lib/load';
import { choosePick, niceStep, type PickTarget } from '../lib/pick';

export const EDGE_COLOUR = '#15181d';
export const JOINT_COLOUR = '#ff8c1a';
export const HOVER_COLOUR = '#ffe14d';
export const SELECT_COLOUR = '#00d5ff';
const PICK_TOLERANCE_PX = 6;

export interface FocusRequest {
  /** Box in the display frame (+Z up). */
  box: Box3;
  key: number;
}

interface Props {
  model: Model;
  hover: PickTarget;
  onHover: (t: PickTarget) => void;
  onPick: (t: PickTarget) => void;
  selectedEdges: ReadonlySet<string>;
  selectedFace: string | null;
  showFrames: boolean;
  focus: FocusRequest;
}

const noRaycast = () => null;

function CameraRig({ focus }: { focus: FocusRequest }) {
  const camera = useThree((s) => s.camera) as PerspectiveCamera;
  const controls = useThree((s) => s.controls) as unknown as { target: Vector3; update(): void } | null;
  useEffect(() => {
    const sphere = focus.box.getBoundingSphere(new Sphere());
    const r = Math.max(sphere.radius, 1);
    const dist = (r / Math.sin(((camera.fov / 2) * Math.PI) / 180)) * 1.15;
    const dir = new Vector3(1, -1.3, 0.9).normalize();
    camera.position.copy(sphere.center).addScaledVector(dir, dist);
    camera.near = Math.max(dist / 2000, 0.01);
    camera.far = dist * 50;
    camera.updateProjectionMatrix();
    if (controls) {
      controls.target.copy(sphere.center);
      controls.update();
    } else {
      camera.lookAt(sphere.center);
    }
  }, [focus, camera, controls]);
  return null;
}

const EdgeLine = memo(function EdgeLine(props: { id: string; points: Vector3[]; colour: string; width: number }) {
  return <Line points={props.points} color={props.colour} lineWidth={props.width} userData={{ edge_id: props.id }} />;
});

function PreviewFrames({ model, size }: { model: Model; size: number }) {
  const items = useMemo(() => {
    const out: { key: string; x: ArrowHelper; z: ArrowHelper; p: Vector3; beamEnd: Vector3 }[] = [];
    for (const j of model.topo.joints) {
      j.preview_frames.forEach((f, i) => {
        const p = new Vector3(...f.point_mm);
        const x = new Vector3(...f.x).normalize();
        const z = new Vector3(...f.z).normalize();
        const ax = new ArrowHelper(x, p, size, 0xe53935, size * 0.25, size * 0.12);
        const az = new ArrowHelper(z, p, size, 0x1e63e9, size * 0.25, size * 0.12);
        for (const o of [ax, az]) o.traverse((c: Object3D) => (c.raycast = noRaycast));
        out.push({ key: `${j.joint_edge_id}-${i}`, x: ax, z: az, p, beamEnd: p.clone().addScaledVector(z, -size) });
      });
    }
    return out;
  }, [model, size]);
  useEffect(
    () => () => {
      for (const it of items) for (const a of [it.x, it.z]) a.dispose();
    },
    [items],
  );
  return (
    <>
      {items.map((it) => (
        <group key={it.key}>
          <primitive object={it.x} />
          <primitive object={it.z} />
          <Line points={[it.p, it.beamEnd]} color="#1e63e9" lineWidth={1.5} dashed dashSize={size * 0.08} gapSize={size * 0.06} raycast={noRaycast} />
        </group>
      ))}
    </>
  );
}

export function Viewer({ model, hover, onHover, onPick, selectedEdges, selectedFace, showFrames, focus }: Props) {
  const size = model.displayBox.getSize(new Vector3());
  const radius = Math.max(size.length() / 2, 1);
  const step = niceStep(Math.max(size.x, size.y, size.z));
  const centre = model.displayBox.getCenter(new Vector3());
  const depthSlack = Math.max(radius * 0.01, 0.2);

  // Face colours: solid colour, lifted on hover / selection.
  useEffect(() => {
    for (const [id, mat] of model.faceMaterials) {
      mat.emissive = new Color(id === selectedFace ? '#0a5d6b' : hover?.kind === 'face' && hover.id === id ? '#5a5016' : '#000000');
    }
  }, [model, hover, selectedFace]);

  const edges = useMemo(() => [...model.edgePoints.entries()], [model]);
  const jointEdges = useMemo(
    () => new Set(model.topo.edges.filter((e) => e.joint_edge_id).map((e) => e.edge_id)),
    [model],
  );

  const pick = (e: ThreeEvent<PointerEvent | MouseEvent>) => choosePick(e.intersections, depthSlack);

  return (
    <Canvas
      camera={{ fov: 40, near: 0.1, far: 1e6, position: [radius * 2, -radius * 2, radius * 2] }}
      onCreated={(s) => {
        (s.raycaster.params as unknown as Record<string, unknown>).Line2 = { threshold: PICK_TOLERANCE_PX };
      }}
    >
      <color attach="background" args={['#262a31']} />
      <hemisphereLight args={['#ffffff', '#55606e', 1.6]} />
      <directionalLight position={[1, -2, 3]} intensity={1.6} />
      <directionalLight position={[-2, 1, -1]} intensity={0.5} />
      <OrbitControls makeDefault enableDamping={false} />
      <CameraRig focus={focus} />
      <group
        matrixAutoUpdate={false}
        matrix={model.upMatrix}
        onPointerMove={(e) => {
          e.stopPropagation();
          onHover(pick(e));
        }}
        onPointerOut={() => onHover(null)}
        onClick={(e) => {
          e.stopPropagation();
          if (e.delta > 4) return; // a drag of the orbit controls, not a click
          onPick(pick(e));
        }}
      >
        <primitive object={model.scene} />
        {edges.map(([id, pts]) => {
          const joint = jointEdges.has(id);
          const sel = selectedEdges.has(id);
          const hov = hover?.kind === 'edge' && hover.id === id;
          const colour = sel ? SELECT_COLOUR : hov ? HOVER_COLOUR : joint ? JOINT_COLOUR : EDGE_COLOUR;
          const width = sel || hov ? 4 : joint ? 3 : 1.5;
          return <EdgeLine key={id} id={id} points={pts} colour={colour} width={width} />;
        })}
        {showFrames && <PreviewFrames model={model} size={Math.max(radius * 0.15, 2)} />}
      </group>
      <Grid
        position={[centre.x, centre.y, model.displayBox.min.z - radius * 0.02]}
        rotation={[Math.PI / 2, 0, 0]}
        args={[step * 40, step * 40]}
        cellSize={step}
        sectionSize={step * 10}
        cellColor="#4a515c"
        sectionColor="#6f7a88"
        fadeDistance={radius * 20}
        infiniteGrid
        raycast={noRaycast}
      />
      <GizmoHelper alignment="bottom-right" margin={[70, 70]}>
        <GizmoViewport axisColors={['#e53935', '#43a047', '#1e63e9']} labelColor="#ffffff" />
      </GizmoHelper>
    </Canvas>
  );
}
