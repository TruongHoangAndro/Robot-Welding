import { GizmoHelper, GizmoViewport, Grid, Line, OrbitControls } from '@react-three/drei';
import { Canvas, useThree } from '@react-three/fiber';
import { useEffect, useMemo } from 'react';
import { BufferGeometry, Color, Float32BufferAttribute, Group, Points, PointsMaterial, Vector3, type Object3D, type PerspectiveCamera } from 'three';
import type { URDFRobot } from 'urdf-loader';
import { setJoints } from '../lib/robot';
import { attachCellScene, type CellNode } from './CellScene';
import { STATUS_COLOURS, type Marker } from '../lib/simulation';
import { poseMmToMatrix, SCENE_UNITS_PER_MM, type PoseMm } from '../lib/units';

export interface TrailPoint {
  /** robot root (world) frame, scene units */
  p: Vector3;
  weld: boolean;
}

interface Props {
  robot: URDFRobot;
  /** part display model (D20 faces, part frame, mm) */
  partScene: Object3D | null;
  flangeTPart: PoseMm;
  markers: Marker[];
  trail: TrailPoint[];
  jointNames: string[];
  q7: number[];
  /** camera target and offset of the camera from it, scene units; a new object re-aims the camera */
  focus: { target: Vector3; offset: Vector3 };
  /** cell scene manifest objects (static under world, rotating under positioner_flange); empty = none shown */
  cell?: CellNode[];
}

function CameraRig({ focus }: { focus: { target: Vector3; offset: Vector3 } }) {
  const { target, offset } = focus;
  const camera = useThree((s) => s.camera) as PerspectiveCamera;
  const controls = useThree((s) => s.controls) as unknown as { target: Vector3; update(): void } | null;
  useEffect(() => {
    camera.up.set(0, 0, 1);
    camera.position.copy(target).add(offset);
    camera.near = 0.01;
    camera.far = 100;
    camera.updateProjectionMatrix();
    if (controls) {
      controls.target.copy(target);
      controls.update();
    } else camera.lookAt(target);
  }, [camera, controls, target, offset]);
  return null;
}

/**
 * Part (D20 faces) and seam point markers attached to the ``positioner_flange`` link through ``flange_T_part``, so
 * they turn with θ. Inside, everything stays in mm (part frame); the one scale factor comes from units.ts.
 */
function usePartOnFlange(robot: URDFRobot, partScene: Object3D | null, flangeTPart: PoseMm, markers: Marker[]) {
  useEffect(() => {
    const flange = robot.links['positioner_flange'];
    const part = new Group();
    part.name = 'part (flange_T_part)';
    part.matrixAutoUpdate = false;
    part.matrix.copy(poseMmToMatrix(flangeTPart));
    const mm = new Group();
    mm.scale.setScalar(SCENE_UNITS_PER_MM);
    part.add(mm);
    if (partScene) mm.add(partScene);
    if (markers.length) {
      const geo = new BufferGeometry();
      geo.setAttribute('position', new Float32BufferAttribute(markers.flatMap((m) => m.pMm), 3));
      geo.setAttribute('color', new Float32BufferAttribute(markers.flatMap((m) => new Color(STATUS_COLOURS[m.status] ?? '#ffffff').toArray()), 3));
      const pts = new Points(geo, new PointsMaterial({ size: 7, sizeAttenuation: false, vertexColors: true, depthTest: false }));
      pts.renderOrder = 10;
      mm.add(pts);
    }
    flange.add(part);
    return () => {
      flange.remove(part);
      if (partScene) mm.remove(partScene);
    };
  }, [robot, partScene, flangeTPart, markers]);
}

export function SimViewer({ robot, partScene, flangeTPart, markers, trail, jointNames, q7, focus, cell }: Props) {
  usePartOnFlange(robot, partScene, flangeTPart, markers);
  useEffect(() => (cell?.length ? attachCellScene(robot, cell) : undefined), [robot, cell]);
  useEffect(() => setJoints(robot, jointNames, q7), [robot, jointNames, q7]);
  const trailLine = useMemo(() => {
    if (trail.length < 2) return null;
    return {
      points: trail.map((t) => t.p),
      colors: trail.map((t) => new Color(t.weld ? '#ff8c1a' : '#8f99a6')),
    };
  }, [trail]);
  return (
    <Canvas camera={{ fov: 40, near: 0.01, far: 100, position: [2, -2, 1.5] }} onCreated={(s) => s.camera.up.set(0, 0, 1)}>
      <color attach="background" args={['#262a31']} />
      <hemisphereLight args={['#ffffff', '#55606e', 1.5]} />
      <directionalLight position={[2, -3, 4]} intensity={1.6} />
      <directionalLight position={[-3, 2, 1]} intensity={0.5} />
      <OrbitControls makeDefault enableDamping={false} />
      <CameraRig focus={focus} />
      <primitive object={robot} />
      {trailLine && <Line points={trailLine.points} vertexColors={trailLine.colors} lineWidth={2} />}
      <Grid rotation={[Math.PI / 2, 0, 0]} args={[4, 4]} cellSize={0.1} sectionSize={1} cellColor="#4a515c" sectionColor="#6f7a88" fadeDistance={12} infiniteGrid />
      <GizmoHelper alignment="bottom-right" margin={[70, 70]}>
        <GizmoViewport axisColors={['#e53935', '#43a047', '#1e63e9']} labelColor="#ffffff" />
      </GizmoHelper>
    </Canvas>
  );
}
