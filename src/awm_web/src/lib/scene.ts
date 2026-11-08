/**
 * S4 static cell view (G1.20, close-out M4): objects of cell/model/scene_manifest.json served by GET /cell/scene.
 * ``parent_frame`` world = static, positioner_flange = turns with θ (pose given at θ = 0). Poses in mm; display only.
 */
import type { PoseMm } from './units';

export interface SceneObject {
  object_id: string;
  role: string;
  component: string;
  parent_frame: string;
  pose_mm: PoseMm;
  collision: string;
  mesh_url: string;
}

export interface ScenePayload {
  unit: string;
  created_at?: string | null;
  note?: string | null;
  objects: SceneObject[];
}

/** Objects by the frame they hang under; any other parent frame is listed and not drawn. */
export function splitScene(objects: SceneObject[]): { world: SceneObject[]; flange: SceneObject[]; unknown: SceneObject[] } {
  const out = { world: [] as SceneObject[], flange: [] as SceneObject[], unknown: [] as SceneObject[] };
  for (const o of objects) (o.parent_frame === 'world' ? out.world : o.parent_frame === 'positioner_flange' ? out.flange : out.unknown).push(o);
  return out;
}
