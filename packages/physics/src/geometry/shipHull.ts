import { loftStations, type Station } from './loft';
import type { TriangleMesh } from './mesh';

/**
 * Parametric displacement-ship hull. Generator frame: x forward from midship, y to starboard,
 * z down with the design waterline at z = 0 (keel at z = draft, deck at z = draft − depth).
 *
 * Waterlines have a parallel middle body that fairs into each end with a power law; sections are
 * superellipses whose fullness drops toward the ends. The forward and aft lengths of parallel
 * body are solved so that the underwater body matches a target block coefficient and LCB.
 */
export interface ShipHullParams {
  length: number;
  beam: number;
  draft: number;
  depth: number;
  blockCoefficient: number;
  /** Longitudinal centre of buoyancy, forward of midship (m). */
  lcb: number;
  /** Section exponent at midship (large = boxy, about 0.99 midship coefficient at 10). */
  midshipExponent?: number;
  /** Waterline end exponent (2 = parabolic entrance). */
  endExponent?: number;
  stations?: number;
  pointsBelow?: number;
  pointsAbove?: number;
}

interface Shape {
  xf: number; // parallel body fraction forward (0..1 of half length)
  xa: number; // parallel body fraction aft
}

function waterlineHalfBreadthFactor(xi: number, shape: Shape, p: number): number {
  const a = Math.abs(xi);
  const x0 = xi >= 0 ? shape.xf : shape.xa;
  if (a <= x0) return 1;
  const t = Math.min(1, (a - x0) / (1 - x0));
  return Math.max(0, 1 - t ** p);
}

function sectionExponent(w: number, mMid: number): number {
  return 2 + (mMid - 2) * w * w;
}

/** Superellipse section: half-breadth fraction at depth fraction ζ ∈ [0, 1]. */
function sectionFactor(zeta: number, mExp: number): number {
  const z = Math.min(1, Math.max(0, zeta));
  return (1 - z ** mExp) ** (1 / mExp);
}

const cmCache = new Map<number, number>();
function sectionAreaCoefficient(mExp: number): number {
  const key = Math.round(mExp * 1000);
  const hit = cmCache.get(key);
  if (hit !== undefined) return hit;
  const n = 200;
  let s = 0;
  for (let i = 0; i < n; i++) s += sectionFactor((i + 0.5) / n, mExp);
  const v = s / n;
  cmCache.set(key, v);
  return v;
}

function bodyStats(p: ShipHullParams, shape: Shape): { cb: number; lcb: number } {
  const mMid = p.midshipExponent ?? 10;
  const pe = p.endExponent ?? 2;
  const n = 400;
  let v = 0;
  let mx = 0;
  for (let i = 0; i < n; i++) {
    const xi = -1 + (2 * (i + 0.5)) / n;
    const w = waterlineHalfBreadthFactor(xi, shape, pe);
    const area = w * sectionAreaCoefficient(sectionExponent(w, mMid));
    v += area;
    mx += area * xi;
  }
  // v/n is the mean of w·Cm over the length = Cb
  return { cb: v / n, lcb: (mx / Math.max(v, 1e-12)) * (p.length / 2) };
}

/** Solve the forward and aft parallel-body fractions for the target Cb and LCB (2-D Newton). */
export function solveShipShape(p: ShipHullParams): Shape {
  let s: Shape = { xf: 0.3, xa: 0.3 };
  for (let it = 0; it < 50; it++) {
    const f = bodyStats(p, s);
    const r0 = f.cb - p.blockCoefficient;
    const r1 = (f.lcb - p.lcb) / p.length;
    if (Math.abs(r0) < 1e-7 && Math.abs(r1) < 1e-7) break;
    const h = 1e-4;
    const fx = bodyStats(p, { xf: s.xf + h, xa: s.xa });
    const fa = bodyStats(p, { xf: s.xf, xa: s.xa + h });
    const j00 = (fx.cb - f.cb) / h;
    const j01 = (fa.cb - f.cb) / h;
    const j10 = (fx.lcb - f.lcb) / p.length / h;
    const j11 = (fa.lcb - f.lcb) / p.length / h;
    const det = j00 * j11 - j01 * j10;
    if (Math.abs(det) < 1e-14) break;
    const dxf = (-r0 * j11 + r1 * j01) / det;
    const dxa = (-r1 * j00 + r0 * j10) / det;
    s = {
      xf: Math.min(0.95, Math.max(0, s.xf + dxf)),
      xa: Math.min(0.95, Math.max(0, s.xa + dxa)),
    };
  }
  return s;
}

export interface ShipHullGeometry {
  mesh: TriangleMesh;
  shape: Shape;
}

export function createShipHull(p: ShipHullParams): ShipHullGeometry {
  const shape = solveShipShape(p);
  const mMid = p.midshipExponent ?? 10;
  const pe = p.endExponent ?? 2;
  const ns = p.stations ?? 41;
  const nb = p.pointsBelow ?? 8;
  const na = p.pointsAbove ?? 2;
  const stations: Station[] = [];
  const zDeck = p.draft - p.depth;
  for (let i = 0; i < ns; i++) {
    // cosine spacing concentrates stations at the ends where the shape changes fastest
    const xi = -Math.cos((Math.PI * i) / (ns - 1));
    const w = waterlineHalfBreadthFactor(xi, shape, pe);
    const mExp = sectionExponent(w, mMid);
    const half = (p.beam / 2) * w;
    const stbd: Array<[number, number]> = [];
    for (let k = 0; k < na; k++) {
      const z = zDeck + ((0 - zDeck) * k) / na;
      stbd.push([half, z]);
    }
    for (let k = 0; k <= nb; k++) {
      // cluster points near the bilge
      const zeta = Math.sin((Math.PI / 2) * (k / nb));
      stbd.push([half * sectionFactor(zeta, mExp), p.draft * zeta]);
    }
    const ring: Array<[number, number]> = [...stbd];
    for (let k = stbd.length - 2; k >= 0; k--) {
      const pt = stbd[k] as [number, number];
      ring.push([-pt[0], pt[1]]);
    }
    stations.push({ x: (xi * p.length) / 2, ring });
  }
  return { mesh: loftStations(stations), shape };
}
