import type { EnvironmentSampler } from '../core/environment';
import type { TriangleMesh } from '../geometry/mesh';
import type { Quat } from '../math/quat';
import type { Vec3 } from '../math/vec3';

/**
 * Hydrostatic (and, in waves, nonlinear Froude–Krylov) loads by integrating the pressure
 * p = ρ g h over the wetted part of a closed hull mesh, where h is the depth of a point below the
 * local free surface. Each triangle is clipped against the surface into 0–2 fully wetted triangles
 * (Kerner 2015). Within a triangle the pressure is linear in position, so force and moment are
 * integrated exactly for that linear field.
 */
export interface HydroResult {
  /** Pressure force on the hull, world frame (N). */
  force: Vec3;
  /** Moment of the pressure force about the CG, world frame (N·m). */
  moment: Vec3;
  /** Wetted surface area (m²). */
  wettedArea: number;
  /** Equivalent displaced volume, vertical force / (ρ g) (m³). */
  volume: number;
  /** Centre of the wetted area, world frame offset from the CG (m). */
  wettedCentroid: Vec3;
  /** Deepest submerged point below the local surface (m, ≤ 0 when airborne). */
  maxDepth: number;
  /** Fraction of hull vertices below the surface. */
  submergedVertexFraction: number;
}

export const newHydroResult = (): HydroResult => ({
  force: { x: 0, y: 0, z: 0 },
  moment: { x: 0, y: 0, z: 0 },
  wettedArea: 0,
  volume: 0,
  wettedCentroid: { x: 0, y: 0, z: 0 },
  maxDepth: 0,
  submergedVertexFraction: 0,
});

/** Preallocated buffers for one mesh. */
export interface HydroScratch {
  world: Float64Array; // world offsets from the CG, xyz per vertex
  depth: Float64Array; // depth below the local surface per vertex
}

export function createHydroScratch(mesh: TriangleMesh): HydroScratch {
  const n = mesh.positions.length / 3;
  return { world: new Float64Array(n * 3), depth: new Float64Array(n) };
}

interface Accum {
  fx: number;
  fy: number;
  fz: number;
  mx: number;
  my: number;
  mz: number;
  area: number;
  cx: number;
  cy: number;
  cz: number;
}

/**
 * Integrate pressure over one fully wetted triangle with vertex offsets r_i (from the CG) and
 * depths h_i ≥ 0. Force on the body is −∫p n dA; moment is −(∫p r dA) × n, with
 * ∫ p r dA = (A/12)(Σ p_i r_i + Σp_i Σr_i) for linear p and r.
 */
function integrateTri(
  acc: Accum,
  ax: number,
  ay: number,
  az: number,
  ha: number,
  bx: number,
  by: number,
  bz: number,
  hb: number,
  cx: number,
  cy: number,
  cz: number,
  hc: number,
  rhoG: number,
): void {
  const e1x = bx - ax;
  const e1y = by - ay;
  const e1z = bz - az;
  const e2x = cx - ax;
  const e2y = cy - ay;
  const e2z = cz - az;
  // n·A·2 (outward for an outward-oriented mesh)
  const nx2 = e1y * e2z - e1z * e2y;
  const ny2 = e1z * e2x - e1x * e2z;
  const nz2 = e1x * e2y - e1y * e2x;
  const area2 = Math.hypot(nx2, ny2, nz2);
  if (area2 < 1e-14) return;
  const area = area2 / 2;
  const pa = rhoG * ha;
  const pb = rhoG * hb;
  const pc = rhoG * hc;
  const pMeanTimesArea = ((pa + pb + pc) / 3) * area;
  const ux = nx2 / area2;
  const uy = ny2 / area2;
  const uz = nz2 / area2;
  acc.fx -= pMeanTimesArea * ux;
  acc.fy -= pMeanTimesArea * uy;
  acc.fz -= pMeanTimesArea * uz;
  const sp = pa + pb + pc;
  const k = area / 12;
  const prx = k * (pa * ax + pb * bx + pc * cx + sp * (ax + bx + cx));
  const pry = k * (pa * ay + pb * by + pc * cy + sp * (ay + by + cy));
  const prz = k * (pa * az + pb * bz + pc * cz + sp * (az + bz + cz));
  // M = −(∫p r) × n
  acc.mx -= pry * uz - prz * uy;
  acc.my -= prz * ux - prx * uz;
  acc.mz -= prx * uy - pry * ux;
  acc.area += area;
  acc.cx += area * (ax + bx + cx) / 3;
  acc.cy += area * (ay + by + cy) / 3;
  acc.cz += area * (az + bz + cz) / 3;
}

/** Clip a partly wet triangle whose odd vertex (wet when count = 1, dry when count = 2) is v0. */
function clipTri(
  acc: Accum,
  count: number,
  v0x: number,
  v0y: number,
  v0z: number,
  h0: number,
  v1x: number,
  v1y: number,
  v1z: number,
  h1: number,
  v2x: number,
  v2y: number,
  v2z: number,
  h2: number,
  rhoG: number,
): void {
  // intersection points on edges v0–v1 and v0–v2 (where h = 0)
  const t1 = h0 / (h0 - h1);
  const t2 = h0 / (h0 - h2);
  const p1x = v0x + (v1x - v0x) * t1;
  const p1y = v0y + (v1y - v0y) * t1;
  const p1z = v0z + (v1z - v0z) * t1;
  const p2x = v0x + (v2x - v0x) * t2;
  const p2y = v0y + (v2y - v0y) * t2;
  const p2z = v0z + (v2z - v0z) * t2;
  if (count === 1) {
    // only v0 is wet: triangle v0, p1, p2
    integrateTri(acc, v0x, v0y, v0z, h0, p1x, p1y, p1z, 0, p2x, p2y, p2z, 0, rhoG);
  } else {
    // v0 is dry: wet quad p1, v1, v2, p2
    integrateTri(acc, p1x, p1y, p1z, 0, v1x, v1y, v1z, h1, v2x, v2y, v2z, h2, rhoG);
    integrateTri(acc, p1x, p1y, p1z, 0, v2x, v2y, v2z, h2, p2x, p2y, p2z, 0, rhoG);
  }
}

/**
 * Compute hydrostatic loads for a mesh (body frame, origin at the CG) placed at `position`
 * (world NED) with attitude `q`, on the free surface given by `env` at time t.
 */
export function computeHydrostatics(
  mesh: TriangleMesh,
  position: Readonly<Vec3>,
  q: Readonly<Quat>,
  env: Pick<EnvironmentSampler, 'elevation' | 'rho' | 'g'>,
  t: number,
  scratch: HydroScratch,
  out: HydroResult,
): HydroResult {
  const p = mesh.positions;
  const idx = mesh.indices;
  const world = scratch.world;
  const depth = scratch.depth;
  const nv = p.length / 3;
  // rotation matrix from the quaternion
  const { w, x, y, z } = q;
  const r00 = 1 - 2 * (y * y + z * z);
  const r01 = 2 * (x * y - w * z);
  const r02 = 2 * (x * z + w * y);
  const r10 = 2 * (x * y + w * z);
  const r11 = 1 - 2 * (x * x + z * z);
  const r12 = 2 * (y * z - w * x);
  const r20 = 2 * (x * z - w * y);
  const r21 = 2 * (y * z + w * x);
  const r22 = 1 - 2 * (x * x + y * y);
  let maxDepth = -Infinity;
  let submerged = 0;
  for (let i = 0; i < nv; i++) {
    const bx = p[3 * i] as number;
    const by = p[3 * i + 1] as number;
    const bz = p[3 * i + 2] as number;
    const wx = r00 * bx + r01 * by + r02 * bz;
    const wy = r10 * bx + r11 * by + r12 * bz;
    const wz = r20 * bx + r21 * by + r22 * bz;
    world[3 * i] = wx;
    world[3 * i + 1] = wy;
    world[3 * i + 2] = wz;
    // NED: z is down; the surface sits at z = −η, so depth below the surface is z + η
    const h = position.z + wz + env.elevation(position.x + wx, position.y + wy, t);
    depth[i] = h;
    if (h > maxDepth) maxDepth = h;
    if (h > 0) submerged++;
  }
  const acc: Accum = { fx: 0, fy: 0, fz: 0, mx: 0, my: 0, mz: 0, area: 0, cx: 0, cy: 0, cz: 0 };
  const rhoG = env.rho * env.g;
  if (submerged > 0) {
    for (let tIdx = 0; tIdx < idx.length; tIdx += 3) {
      const ia = idx[tIdx] as number;
      const ib = idx[tIdx + 1] as number;
      const ic = idx[tIdx + 2] as number;
      const ha = depth[ia] as number;
      const hb = depth[ib] as number;
      const hc = depth[ic] as number;
      const sa = ha > 0;
      const sb = hb > 0;
      const sc = hc > 0;
      const count = (sa ? 1 : 0) + (sb ? 1 : 0) + (sc ? 1 : 0);
      if (count === 0) continue;
      const ax = world[3 * ia] as number;
      const ay = world[3 * ia + 1] as number;
      const az = world[3 * ia + 2] as number;
      const bx = world[3 * ib] as number;
      const by = world[3 * ib + 1] as number;
      const bz = world[3 * ib + 2] as number;
      const cx = world[3 * ic] as number;
      const cy = world[3 * ic + 1] as number;
      const cz = world[3 * ic + 2] as number;
      if (count === 3) {
        integrateTri(acc, ax, ay, az, ha, bx, by, bz, hb, cx, cy, cz, hc, rhoG);
        continue;
      }
      // rotate the vertex order so the odd-one-out is first, keeping the winding
      const odd = count === 1 ? (sa ? 0 : sb ? 1 : 2) : !sa ? 0 : !sb ? 1 : 2;
      if (odd === 0) clipTri(acc, count, ax, ay, az, ha, bx, by, bz, hb, cx, cy, cz, hc, rhoG);
      else if (odd === 1) clipTri(acc, count, bx, by, bz, hb, cx, cy, cz, hc, ax, ay, az, ha, rhoG);
      else clipTri(acc, count, cx, cy, cz, hc, ax, ay, az, ha, bx, by, bz, hb, rhoG);
    }
  }
  out.force.x = acc.fx;
  out.force.y = acc.fy;
  out.force.z = acc.fz;
  out.moment.x = acc.mx;
  out.moment.y = acc.my;
  out.moment.z = acc.mz;
  out.wettedArea = acc.area;
  out.volume = -acc.fz / rhoG;
  const ia = acc.area > 0 ? 1 / acc.area : 0;
  out.wettedCentroid.x = acc.cx * ia;
  out.wettedCentroid.y = acc.cy * ia;
  out.wettedCentroid.z = acc.cz * ia;
  out.maxDepth = submerged > 0 ? maxDepth : Math.min(0, maxDepth);
  out.submergedVertexFraction = submerged / nv;
  return out;
}
