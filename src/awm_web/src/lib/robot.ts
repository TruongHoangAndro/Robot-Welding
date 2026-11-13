/**
 * Robot description in the browser (urdf-loader, ADR-09). The URDF comes from awm_api (rendered like the planner,
 * metres / radians); joint values arrive in degrees and are converted only through ``units.ts``.
 * Joint limits are **not** clamped here (urdf-loader would silently change the played value); values beyond a limit
 * are reported by ``limitViolations`` instead.
 */
import { Group, Matrix4, Mesh, MeshStandardMaterial, type LoadingManager, type Material, type Object3D } from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { degToRad, poseMmToMatrix, radToDeg, type PoseMm } from './units';

export type MeshLoader = (url: string, manager: LoadingManager, material: Material, done: (obj: Object3D, err?: Error) => void) => void;

export const TCP_LINK = 'tcp_laser';
export const FLANGE_LINK = 'positioner_flange';

/** Parse a URDF string (``requireTcp`` false: a cell without head.yaml has no tcp_laser link, S11); without ``loadMesh`` no visual geometry is loaded (kinematics only, tests). */
export function parseRobot(urdf: string, loadMesh?: MeshLoader, requireTcp = true): URDFRobot {
  const loader = new URDFLoader();
  loader.parseVisual = !!loadMesh;
  loader.parseCollision = false;
  loader.loadMeshCb = loadMesh ?? ((_u, _m, _mat, done) => done(new Group()));
  const robot = loader.parse(urdf);
  for (const j of Object.values(robot.joints)) j.ignoreLimits = true;
  for (const l of requireTcp ? [TCP_LINK, FLANGE_LINK] : [FLANGE_LINK]) if (!robot.links[l]) throw new Error(`robot description has no link ${l}`);
  return robot;
}

/** Set joint values (degrees) by name. */
export function setJoints(robot: URDFRobot, names: string[], deg: number[]): void {
  names.forEach((n, i) => {
    const j = robot.joints[n];
    if (!j) throw new Error(`robot description has no joint ${n}`);
    j.setJointValue(degToRad(deg[i]));
  });
  robot.updateMatrixWorld(true);
}

/** Joints whose value is outside the URDF limit (degrees). */
export function limitViolations(robot: URDFRobot, names: string[], deg: number[]): string[] {
  const out: string[] = [];
  names.forEach((n, i) => {
    const j = robot.joints[n];
    if (j && j.jointType === 'revolute') {
      const lo = radToDeg(Number(j.limit.lower));
      const hi = radToDeg(Number(j.limit.upper));
      if (deg[i] < lo - 1e-9 || deg[i] > hi + 1e-9) out.push(`${n} ${deg[i].toFixed(2)}° outside [${lo.toFixed(1)}, ${hi.toFixed(1)}]`);
    }
  });
  return out;
}

/** Transform of a link relative to the robot root (the URDF ``world`` link), scene units. */
export function rootT(robot: URDFRobot, link: string): Matrix4 {
  robot.updateMatrixWorld(true);
  return new Matrix4().copy(robot.matrixWorld).invert().multiply(robot.links[link].matrixWorld);
}

/** ``part_T_tcp`` from the URDF chain: (root_T_flange · flange_T_part)⁻¹ · root_T_tcp, scene units. */
export function tcpInPart(robot: URDFRobot, flangeTPart: PoseMm): Matrix4 {
  const rootTPart = rootT(robot, FLANGE_LINK).multiply(poseMmToMatrix(flangeTPart));
  return rootTPart.invert().multiply(rootT(robot, TCP_LINK));
}

/**
 * Browser mesh loader for the awm_api mesh URLs (bearer token; STL and COLLADA). Display only.
 */
export function authMeshLoader(headers: Record<string, string>): MeshLoader {
  return (url, _manager, material, done) => {
    fetch(url, { headers, cache: 'force-cache' })
      .then(async (r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status} for ${url}`);
        const lower = url.toLowerCase();
        if (lower.endsWith('.stl')) {
          const geo = new STLLoader().parse(await r.arrayBuffer());
          geo.computeVertexNormals();
          const mat = material instanceof MeshStandardMaterial ? material : new MeshStandardMaterial({ color: '#c8c8c8' });
          done(new Mesh(geo, mat));
        } else if (lower.endsWith('.dae')) {
          const dae = new ColladaLoader().parse(await r.text(), url.slice(0, url.lastIndexOf('/') + 1));
          done(dae?.scene ?? new Group());
        } else {
          throw new Error(`unsupported mesh type: ${url}`);
        }
      })
      .catch((e: Error) => done(new Group(), e));
  };
}
