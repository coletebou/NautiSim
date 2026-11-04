import { calmWater } from '../core/environment';
import { meshBounds, type TriangleMesh } from '../geometry/mesh';
import { quatFromEuler, rotateInverse } from '../math/quat';
import { computeHydrostatics, createHydroScratch, newHydroResult } from './hydrostatics';

export interface Equilibrium {
  /** World z of the CG (m, NED: negative when the CG is above the water). */
  z: number;
  /** Trim angle (rad, positive bow up). */
  pitch: number;
  /** Displaced volume at equilibrium (m³). */
  volume: number;
  converged: boolean;
}

export interface EquilibriumOptions {
  rho?: number;
  g?: number;
  solvePitch?: boolean;
  roll?: number;
}

/**
 * Free-floating equilibrium in calm water: heave (and optionally trim) such that buoyancy balances
 * weight and the centre of buoyancy lies under the CG. 2-D Newton with finite-difference Jacobian.
 */
export function solveEquilibrium(mesh: TriangleMesh, mass: number, opts: EquilibriumOptions = {}): Equilibrium {
  const env = calmWater({ ...(opts.rho !== undefined && { rho: opts.rho }), ...(opts.g !== undefined && { g: opts.g }) });
  const scratch = createHydroScratch(mesh);
  const res = newHydroResult();
  const bounds = meshBounds(mesh);
  const L = bounds.max.x - bounds.min.x;
  const roll = opts.roll ?? 0;
  const weight = mass * env.g;
  const residual = (z: number, pitch: number): [number, number, number] => {
    const q = quatFromEuler(roll, pitch, 0);
    computeHydrostatics(mesh, { x: 0, y: 0, z }, q, env, 0, scratch, res);
    const mBody = rotateInverse(q, res.moment);
    return [(res.force.z + weight) / weight, mBody.y / (weight * L), res.volume];
  };
  // bracket the heave first: fully out (z = −max z) to fully in
  let lo = -bounds.max.z; // keel at the surface: no buoyancy, residual > 0
  let hi = -bounds.min.z; // deck at the surface: residual < 0 if it floats
  let z = 0;
  for (let i = 0; i < 80; i++) {
    z = 0.5 * (lo + hi);
    const r = residual(z, 0)[0];
    if (r > 0) lo = z;
    else hi = z;
    if (hi - lo < 1e-6 * L) break;
  }
  let pitch = 0;
  let converged = !opts.solvePitch;
  if (opts.solvePitch) {
    for (let it = 0; it < 40; it++) {
      const [r0, r1] = residual(z, pitch);
      if (Math.abs(r0) < 1e-9 && Math.abs(r1) < 1e-9) {
        converged = true;
        break;
      }
      const hz = 1e-4 * L;
      const hp = 1e-4;
      const [a0, a1] = residual(z + hz, pitch);
      const [b0, b1] = residual(z, pitch + hp);
      const j00 = (a0 - r0) / hz;
      const j01 = (b0 - r0) / hp;
      const j10 = (a1 - r1) / hz;
      const j11 = (b1 - r1) / hp;
      const det = j00 * j11 - j01 * j10;
      if (Math.abs(det) < 1e-18) break;
      let dz = (-r0 * j11 + r1 * j01) / det;
      let dp = (-r1 * j00 + r0 * j10) / det;
      // damp large steps
      const maxDp = 0.05;
      if (Math.abs(dp) > maxDp) {
        const k = maxDp / Math.abs(dp);
        dp *= k;
        dz *= k;
      }
      z += dz;
      pitch += dp;
    }
  }
  const volume = residual(z, pitch)[2];
  return { z, pitch, volume, converged };
}

export interface StabilityProperties {
  equilibrium: Equilibrium;
  /** Transverse metacentric height from the righting-arm slope (m). */
  gm: number;
  /** Waterplane area from dV/dz (m²). */
  waterplaneArea: number;
  /** Righting arm GZ (m) at a heel angle (rad), at the equilibrium draft and trim. */
  gz: (heel: number) => number;
}

export function stabilityProperties(mesh: TriangleMesh, mass: number, opts: EquilibriumOptions = {}): StabilityProperties {
  const eq = solveEquilibrium(mesh, mass, { ...opts, solvePitch: opts.solvePitch ?? true });
  const env = calmWater({ ...(opts.rho !== undefined && { rho: opts.rho }), ...(opts.g !== undefined && { g: opts.g }) });
  const scratch = createHydroScratch(mesh);
  const res = newHydroResult();
  const weight = mass * env.g;
  const gz = (heel: number): number => {
    const q = quatFromEuler(heel, eq.pitch, 0);
    computeHydrostatics(mesh, { x: 0, y: 0, z: eq.z }, q, env, 0, scratch, res);
    // righting moment about the body x axis; positive GZ opposes positive heel
    return -rotateInverse(q, res.moment).x / weight;
  };
  const d = 0.5 * (Math.PI / 180);
  const gm = (gz(d) - gz(-d)) / (2 * d);
  const bounds = meshBounds(mesh);
  const dz = 1e-3 * (bounds.max.z - bounds.min.z);
  const vol = (z: number): number => {
    computeHydrostatics(mesh, { x: 0, y: 0, z }, quatFromEuler(0, eq.pitch, 0), env, 0, scratch, res);
    return res.volume;
  };
  const waterplaneArea = (vol(eq.z + dz) - vol(eq.z - dz)) / (2 * dz);
  return { equilibrium: eq, gm, waterplaneArea, gz };
}
