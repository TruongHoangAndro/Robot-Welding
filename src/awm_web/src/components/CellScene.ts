/**
 * S4 static cell view (G1.20, close-out M4): meshes of cell/model/scene_manifest.json (export-scene, CP-60) as
 * three.js nodes. Each node = pose (``parent_T_component`` at θ = 0, mm → scene units through units.ts) with the STL
 * (component frame, mm) inside one scale group. Static objects are attached to the ``world`` link, rotating objects to
 * ``positioner_flange`` so they turn with θ. Display only: nothing here is used for planning or collision.
 */
import { DoubleSide, Group, Mesh, MeshStandardMaterial } from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import type { URDFRobot } from 'urdf-loader';
import { meshCacheMode, splitScene, type SceneObject, type ScenePayload } from '../lib/scene';
import { poseMmToMatrix, SCENE_UNITS_PER_MM } from '../lib/units';

export const SCENE_COLOUR = { world: '#9aa3ad', positioner_flange: '#4c8dff' } as const;

export interface CellNode {
  object: SceneObject;
  node: Group;
}

function nodeFor(o: SceneObject, mesh: Mesh): Group {
  const g = new Group();
  g.name = `cell ${o.object_id} (${o.component})`;
  g.matrixAutoUpdate = false;
  g.matrix.copy(poseMmToMatrix(o.pose_mm));
  const mm = new Group();
  mm.scale.setScalar(SCENE_UNITS_PER_MM);
  mm.add(mesh);
  g.add(mm);
  return g;
}

/** Load every drawable object; failures are returned as text (the view still shows the rest). */
export async function loadCellScene(scene: ScenePayload, headers: Record<string, string>): Promise<{ nodes: CellNode[]; errors: string[] }> {
  const { world, flange, unknown } = splitScene(scene.objects);
  const errors = unknown.map((o) => `${o.object_id}: parent frame ${o.parent_frame} not drawn`);
  const nodes: CellNode[] = [];
  await Promise.all(
    [...world, ...flange].map(async (o) => {
      try {
        const r = await fetch(o.mesh_url, { headers, cache: meshCacheMode(o.mesh_url) });
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const geo = new STLLoader().parse(await r.arrayBuffer());
        geo.computeVertexNormals();
        const colour = o.parent_frame === 'world' ? SCENE_COLOUR.world : SCENE_COLOUR.positioner_flange;
        const mat = new MeshStandardMaterial({ color: colour, roughness: 0.8, side: DoubleSide, transparent: true, opacity: 0.85 });
        nodes.push({ object: o, node: nodeFor(o, new Mesh(geo, mat)) });
      } catch (e) {
        errors.push(`${o.object_id}: ${(e as Error).message}`);
      }
    }),
  );
  return { nodes, errors };
}

/** Attach the nodes under their links (``world`` = the robot root when it has no such link); returns a detach. */
export function attachCellScene(robot: URDFRobot, nodes: CellNode[]): () => void {
  const done: [Group, { remove(o: Group): unknown }][] = [];
  for (const n of nodes) {
    const parent = n.object.parent_frame === 'world' ? robot.links['world'] ?? robot : robot.links['positioner_flange'];
    if (!parent) continue;
    parent.add(n.node);
    done.push([n.node, parent]);
  }
  return () => done.forEach(([node, parent]) => parent.remove(node));
}
