import { Box3, Color, DoubleSide, Group, Matrix4, MeshStandardMaterial, Vector3 } from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { buildGlbIndex, checkGlbAsset, consistencyWarnings } from './glbIndex';
import { indexTopology, parseTopology, ViewModelError, type Topology, type TopologyIndex } from './topology';
import { upAxisToZ } from './upAxis';

/** Colours for solids B1, B2, … (cycled), chosen to stay apart from the edge / joint / highlight colours. */
export const SOLID_COLOURS = ['#7fa7d9', '#9fd39b', '#d9a77f', '#c39bd3', '#7fd3c9', '#d3cf7f', '#d38f9b', '#a3a8b4'];

export interface Model {
  topo: Topology;
  tidx: TopologyIndex;
  /** glTF scene (faces only; edge Line nodes are taken out and drawn as fat lines). Part frame, mm. */
  scene: Group;
  faceMaterials: Map<string, MeshStandardMaterial>;
  /** Edge polylines in the part frame, mm, as stored in view.glb. Display only, never a weld path. */
  edgePoints: Map<string, Vector3[]>;
  solidColour: Map<string, string>;
  upMatrix: Matrix4;
  /** Bounding box of the model after the display rotation (+Z up). */
  displayBox: Box3;
  warnings: string[];
  label: string;
}

function parseGlb(buf: ArrayBuffer) {
  return new Promise<import('three/examples/jsm/loaders/GLTFLoader.js').GLTF>((ok, fail) =>
    new GLTFLoader().parse(buf, '', ok, (e) => fail(new ViewModelError(`view.glb could not be parsed (${String((e as unknown as { message?: string })?.message ?? e)})`))),
  );
}

export async function buildModel(glbBuf: ArrayBuffer, topoText: string, label: string): Promise<Model> {
  const topo = parseTopology(topoText);
  const tidx = indexTopology(topo);
  const gltf = await parseGlb(glbBuf);
  try {
    checkGlbAsset(gltf.asset as never);
  } catch (e) {
    throw new ViewModelError((e as Error).message);
  }
  const scene = gltf.scene;
  scene.updateMatrixWorld(true);
  const glb = buildGlbIndex(scene);
  const warnings = consistencyWarnings(glb, topo);

  const solidColour = new Map<string, string>();
  topo.solids.forEach((s, i) => solidColour.set(s.solid_id, SOLID_COLOURS[i % SOLID_COLOURS.length]));

  const faceMaterials = new Map<string, MeshStandardMaterial>();
  for (const [id, mesh] of glb.faces) {
    const sid = (mesh.userData.solid_id as string | null) ?? tidx.faces.get(id)?.solid_id ?? '';
    const mat = new MeshStandardMaterial({
      color: new Color(solidColour.get(sid) ?? '#b0b0b0'),
      side: DoubleSide,
      roughness: 0.7,
      metalness: 0.05,
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 1,
    });
    mesh.material = mat;
    faceMaterials.set(id, mat);
  }

  const edgePoints = new Map<string, Vector3[]>();
  for (const [id, line] of glb.edges) {
    const pos = line.geometry.getAttribute('position');
    const pts: Vector3[] = [];
    for (let i = 0; i < pos.count; i++) pts.push(new Vector3().fromBufferAttribute(pos, i).applyMatrix4(line.matrixWorld));
    edgePoints.set(id, pts);
    line.removeFromParent();
  }

  const upMatrix = upAxisToZ(topo.source.up_axis);
  const partBox = new Box3();
  for (const s of topo.solids) {
    partBox.expandByPoint(new Vector3(...s.bbox_min_mm));
    partBox.expandByPoint(new Vector3(...s.bbox_max_mm));
  }
  if (partBox.isEmpty()) partBox.setFromObject(scene);
  const displayBox = partBox.clone().applyMatrix4(upMatrix);
  return { topo, tidx, scene, faceMaterials, edgePoints, solidColour, upMatrix, displayBox, warnings, label };
}

async function fetchOk(url: string, headers?: Record<string, string>): Promise<Response> {
  let r: Response;
  try {
    r = await fetch(url, { cache: 'no-store', headers });
  } catch (e) {
    throw new ViewModelError(`cannot fetch ${url} (${(e as Error).message})`);
  }
  if (!r.ok) {
    const body = (await r.text().catch(() => '')).slice(0, 200);
    throw new ViewModelError(`cannot load ${url}: HTTP ${r.status}${body ? ` — ${body}` : ''}`);
  }
  return r;
}

/** Load view.glb + topology.json from a base URL; ``headers`` carries the awm_api bearer token when needed. */
export async function loadFromUrl(base: string, headers?: Record<string, string>): Promise<Model> {
  const [g, t] = await Promise.all([fetchOk(`${base}view.glb`, headers), fetchOk(`${base}topology.json`, headers)]);
  return buildModel(await g.arrayBuffer(), await t.text(), base);
}

export async function loadFromFiles(glb: File, topo: File): Promise<Model> {
  return buildModel(await glb.arrayBuffer(), await topo.text(), `${glb.name} + ${topo.name} (local files)`);
}
