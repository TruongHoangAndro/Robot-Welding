import { OrbitControls } from '@react-three/drei';
import { Canvas, useThree } from '@react-three/fiber';
import { useEffect, useMemo } from 'react';
import { Box3, Color, DoubleSide, Group, Mesh, MeshStandardMaterial, PerspectiveCamera, Sphere, Vector3 } from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { ROLE_COLOUR, type Assignable } from '../lib/roles';
import type { UpAxis } from '../lib/topology';
import { upAxisToZ } from '../lib/upAxis';

/** Load cell_view.glb (one node per leaf component, extras.index = inspect-assembly index; file frame, mm). */
export async function loadCellView(url: string, headers: Record<string, string>): Promise<Group> {
  const r = await fetch(url, { headers, cache: 'no-store' });
  if (!r.ok) throw new Error(`cell_view.glb: HTTP ${r.status}`);
  const buf = await r.arrayBuffer();
  return new Promise((ok, fail) => new GLTFLoader().parse(buf, '', (g) => ok(g.scene), (e) => fail(new Error(String(e)))));
}

function Fit({ box }: { box: Box3 }) {
  const camera = useThree((s) => s.camera) as PerspectiveCamera;
  const controls = useThree((s) => s.controls) as unknown as { target: Vector3; update(): void } | null;
  useEffect(() => {
    const s = box.getBoundingSphere(new Sphere());
    const dist = (Math.max(s.radius, 1) / Math.sin((camera.fov / 2) * (Math.PI / 180))) * 1.1;
    camera.position.copy(s.center).addScaledVector(new Vector3(1, -1.3, 0.9).normalize(), dist);
    camera.near = dist / 2000;
    camera.far = dist * 50;
    camera.updateProjectionMatrix();
    if (controls) {
      controls.target.copy(s.center);
      controls.update();
    }
  }, [box, camera, controls]);
  return null;
}

/** Whole cell (or job file) coloured by effective role; click a component to select it in the tree. Display only. */
export function CellView(props: { scene: Group; upAxis: UpAxis; roles: Assignable[]; selected: number | null; onSelect: (index: number) => void }) {
  const { scene, upAxis, roles, selected } = props;
  const up = useMemo(() => upAxisToZ(upAxis), [upAxis]);
  useEffect(() => {
    scene.traverse((o) => {
      if (!(o instanceof Mesh)) return;
      const idx = (o.parent?.userData?.index ?? o.userData?.index) as number | undefined;
      const role = idx !== undefined ? roles[idx] : undefined;
      const sel = idx !== undefined && idx === selected;
      o.material = new MeshStandardMaterial({
        color: new Color(sel ? '#00d5ff' : role ? ROLE_COLOUR[role] : '#b0b0b0'),
        side: DoubleSide,
        transparent: role === 'exclude',
        opacity: role === 'exclude' ? 0.15 : 1,
        roughness: 0.7,
      });
    });
  }, [scene, roles, selected]);
  const box = useMemo(() => new Box3().setFromObject(scene).applyMatrix4(up), [scene, up]);
  return (
    <Canvas camera={{ fov: 40, near: 1, far: 1e7 }}>
      <color attach="background" args={['#262a31']} />
      <hemisphereLight args={['#ffffff', '#55606e', 1.6]} />
      <directionalLight position={[1, -2, 3]} intensity={1.4} />
      <OrbitControls makeDefault enableDamping={false} />
      <Fit box={box} />
      <group
        matrixAutoUpdate={false}
        matrix={up}
        onClick={(e) => {
          e.stopPropagation();
          if (e.delta > 4) return;
          const idx = (e.object.parent?.userData?.index ?? e.object.userData?.index) as number | undefined;
          if (idx !== undefined) props.onSelect(idx);
        }}
      >
        <primitive object={scene} />
      </group>
    </Canvas>
  );
}
