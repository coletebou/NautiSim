import type { Vec3 } from '../math/vec3';

/** Closed, outward-oriented triangle mesh. Positions are xyz triples, indices are vertex triples. */
export interface TriangleMesh {
  readonly positions: Float64Array;
  readonly indices: Uint32Array;
}

export const vertexCount = (m: TriangleMesh): number => m.positions.length / 3;
export const triangleCount = (m: TriangleMesh): number => m.indices.length / 3;

export function meshFromArrays(positions: number[], indices: number[]): TriangleMesh {
  return { positions: Float64Array.from(positions), indices: Uint32Array.from(indices) };
}

/** Signed volume by the divergence theorem (positive when normals point outward). */
export function meshVolume(m: TriangleMesh): number {
  return meshVolumeAndCentroid(m).volume;
}

export function meshVolumeAndCentroid(m: TriangleMesh): { volume: number; centroid: Vec3 } {
  const p = m.positions;
  const idx = m.indices;
  let v6 = 0;
  let cx = 0;
  let cy = 0;
  let cz = 0;
  for (let t = 0; t < idx.length; t += 3) {
    const a = 3 * (idx[t] as number);
    const b = 3 * (idx[t + 1] as number);
    const c = 3 * (idx[t + 2] as number);
    const ax = p[a] as number;
    const ay = p[a + 1] as number;
    const az = p[a + 2] as number;
    const bx = p[b] as number;
    const by = p[b + 1] as number;
    const bz = p[b + 2] as number;
    const qx = p[c] as number;
    const qy = p[c + 1] as number;
    const qz = p[c + 2] as number;
    const d = ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx);
    v6 += d;
    cx += d * (ax + bx + qx);
    cy += d * (ay + by + qy);
    cz += d * (az + bz + qz);
  }
  const volume = v6 / 6;
  const k = v6 !== 0 ? 1 / (4 * v6) : 0;
  return { volume, centroid: { x: cx * k, y: cy * k, z: cz * k } };
}

export function meshBounds(m: TriangleMesh): { min: Vec3; max: Vec3 } {
  const p = m.positions;
  const min = { x: Infinity, y: Infinity, z: Infinity };
  const max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (let i = 0; i < p.length; i += 3) {
    const x = p[i] as number;
    const y = p[i + 1] as number;
    const z = p[i + 2] as number;
    if (x < min.x) min.x = x;
    if (y < min.y) min.y = y;
    if (z < min.z) min.z = z;
    if (x > max.x) max.x = x;
    if (y > max.y) max.y = y;
    if (z > max.z) max.z = z;
  }
  return { min, max };
}

export function translateMesh(m: TriangleMesh, d: Readonly<Vec3>): TriangleMesh {
  const p = new Float64Array(m.positions);
  for (let i = 0; i < p.length; i += 3) {
    p[i] = (p[i] as number) + d.x;
    p[i + 1] = (p[i + 1] as number) + d.y;
    p[i + 2] = (p[i + 2] as number) + d.z;
  }
  return { positions: p, indices: m.indices };
}

export function flipMesh(m: TriangleMesh): TriangleMesh {
  const idx = new Uint32Array(m.indices);
  for (let t = 0; t < idx.length; t += 3) {
    const b = idx[t + 1] as number;
    idx[t + 1] = idx[t + 2] as number;
    idx[t + 2] = b;
  }
  return { positions: m.positions, indices: idx };
}

/** Return the mesh with outward normals (flips it when its signed volume is negative). */
export function orientOutward(m: TriangleMesh): TriangleMesh {
  return meshVolume(m) < 0 ? flipMesh(m) : m;
}

/** Axis-aligned box centred at the origin. */
export function boxMesh(length: number, beam: number, height: number): TriangleMesh {
  const x = length / 2;
  const y = beam / 2;
  const z = height / 2;
  const positions = [-x, -y, -z, x, -y, -z, x, y, -z, -x, y, -z, -x, -y, z, x, -y, z, x, y, z, -x, y, z];
  // faces as quads (outward), split into triangles
  const quads = [
    [0, 3, 2, 1], // z = −h/2 (top in NED, normal −z)
    [4, 5, 6, 7], // z = +h/2 (bottom, normal +z)
    [0, 1, 5, 4], // y = −b/2
    [2, 3, 7, 6], // y = +b/2
    [1, 2, 6, 5], // x = +l/2
    [3, 0, 4, 7], // x = −l/2
  ];
  const indices: number[] = [];
  for (const q of quads) {
    const [a, b, c, d] = q as [number, number, number, number];
    indices.push(a, b, c, a, c, d);
  }
  return orientOutward(meshFromArrays(positions, indices));
}
