import { meshFromArrays, orientOutward, type TriangleMesh } from './mesh';

/** One transverse section: a closed ring of (y, z) points at longitudinal position x. */
export interface Station {
  x: number;
  /** Closed polygon in the y-z plane, same point count for every station. */
  ring: ReadonlyArray<readonly [number, number]>;
}

/**
 * Loft a closed hull from stations ordered along x. Consecutive rings are joined with quads and
 * the two end rings are capped with fans from their centroid. Degenerate rings (a stem collapsing
 * to a line) are fine: they produce zero-area triangles. The result is oriented outward.
 */
export function loftStations(stations: readonly Station[]): TriangleMesh {
  if (stations.length < 2) throw new Error('loftStations: need at least two stations');
  const n = (stations[0] as Station).ring.length;
  for (const s of stations) {
    if (s.ring.length !== n) throw new Error('loftStations: all rings need the same point count');
  }
  const positions: number[] = [];
  const indices: number[] = [];
  for (const s of stations) {
    for (const [y, z] of s.ring) positions.push(s.x, y, z);
  }
  const ns = stations.length;
  for (let i = 0; i < ns - 1; i++) {
    const a0 = i * n;
    const b0 = (i + 1) * n;
    for (let j = 0; j < n; j++) {
      const j1 = (j + 1) % n;
      indices.push(a0 + j, b0 + j, b0 + j1, a0 + j, b0 + j1, a0 + j1);
    }
  }
  const cap = (si: number, reverse: boolean): void => {
    const s = stations[si] as Station;
    let cy = 0;
    let cz = 0;
    for (const [y, z] of s.ring) {
      cy += y;
      cz += z;
    }
    const c = positions.length / 3;
    positions.push(s.x, cy / n, cz / n);
    const base = si * n;
    for (let j = 0; j < n; j++) {
      const j1 = (j + 1) % n;
      if (reverse) indices.push(c, base + j1, base + j);
      else indices.push(c, base + j, base + j1);
    }
  };
  cap(0, false);
  cap(ns - 1, true);
  return orientOutward(meshFromArrays(positions, indices));
}
