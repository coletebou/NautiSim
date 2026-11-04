import { loftStations, type Station } from './loft';
import type { TriangleMesh } from './mesh';

/**
 * Prismatic deep-V planing hull with a raked bow. Generator frame: x forward from the transom
 * (x = 0 at the transom, x = length at the stem), y to starboard, z down with the keel at the
 * transom at z = 0. Chine beam and deadrise are constant over the aft "prismatic" part (where
 * Savitsky's equations apply); forward of it the keel rises, the chine narrows and the deadrise
 * warps up toward the stem.
 */
export interface PlaningHullParams {
  length: number;
  /** Chine beam (m). */
  chineBeam: number;
  /** Beam at the sheer (m). */
  sheerBeam: number;
  /** Deadrise at the transom (rad). */
  deadrise: number;
  /** Hull depth from keel to sheer at the transom (m). */
  depth: number;
  /** Fraction of the length from the transom that is prismatic. */
  prismaticFraction?: number;
  /** Rise of the keel at the stem above the transom keel (m). */
  bowRise?: number;
  /** Deadrise at the stem (rad). */
  bowDeadrise?: number;
  stations?: number;
  pointsBottom?: number;
  pointsSide?: number;
}

export function createPlaningHull(p: PlaningHullParams): TriangleMesh {
  const L = p.length;
  const pf = p.prismaticFraction ?? 0.55;
  const bowRise = p.bowRise ?? 0.45 * p.depth;
  const bowDeadrise = p.bowDeadrise ?? Math.min(1.1, p.deadrise * 2.2);
  const ns = p.stations ?? 25;
  const nb = p.pointsBottom ?? 4;
  const nside = p.pointsSide ?? 2;
  const stations: Station[] = [];
  for (let i = 0; i < ns; i++) {
    const x = (L * i) / (ns - 1);
    const f = x / L;
    // forward taper parameter: 0 in the prismatic part, 1 at the stem
    const t = f <= pf ? 0 : (f - pf) / (1 - pf);
    const keelZ = -bowRise * t * t;
    const shape = Math.sqrt(Math.max(0, 1 - t * t)); // elliptical plan-form toward the stem
    const chineHalf = (p.chineBeam / 2) * shape;
    const sheerHalf = (p.sheerBeam / 2) * Math.max(shape, 0) ** 0.8;
    const dr = p.deadrise + (bowDeadrise - p.deadrise) * t;
    const chineZ = keelZ - chineHalf * Math.tan(dr);
    const sheerZ = -p.depth - 0.15 * p.depth * t; // sheer rises slightly forward
    const stbd: Array<[number, number]> = [];
    for (let k = 0; k < nside; k++) {
      const s = k / nside;
      stbd.push([sheerHalf + (chineHalf - sheerHalf) * s, sheerZ + (chineZ - sheerZ) * s]);
    }
    for (let k = 0; k <= nb; k++) {
      const s = k / nb;
      stbd.push([chineHalf * (1 - s), chineZ + (keelZ - chineZ) * s]);
    }
    const ring: Array<[number, number]> = [...stbd];
    for (let k = stbd.length - 2; k >= 0; k--) {
      const pt = stbd[k] as [number, number];
      ring.push([-pt[0], pt[1]]);
    }
    stations.push({ x, ring });
  }
  return loftStations(stations);
}
