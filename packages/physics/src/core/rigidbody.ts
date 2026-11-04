import type { Quat } from '../math/quat';
import type { Vec3 } from '../math/vec3';
import type { DofMask, Dof6, Wrench } from './types';

/**
 * 6-DOF rigid body about the centre of gravity, in Fossen's body-frame form:
 *   (M_RB + M_A) ν̇ + C_RB(ν) ν = τ + g(η)
 * with diagonal rigid-body inertia and diagonal added mass. Gravity is included here; all fluid
 * loads (including added-mass Coriolis, if a hull wants it) come in as τ from force models.
 *
 * State vector layout (13 doubles): [x y z | qw qx qy qz | u v w | p q r].
 */
export const STATE_SIZE = 13;
export const IX = 0;
export const IQ = 3;
export const IV = 7;
export const IW = 10;

export interface BodyProperties {
  mass: number;
  inertia: Readonly<Vec3>;
  addedMass: Readonly<Dof6>;
  g: number;
  mask: DofMask;
}

export function packState(out: Float64Array, p: Readonly<Vec3>, q: Readonly<Quat>, v: Readonly<Vec3>, w: Readonly<Vec3>): Float64Array {
  out[0] = p.x;
  out[1] = p.y;
  out[2] = p.z;
  out[3] = q.w;
  out[4] = q.x;
  out[5] = q.y;
  out[6] = q.z;
  out[7] = v.x;
  out[8] = v.y;
  out[9] = v.z;
  out[10] = w.x;
  out[11] = w.y;
  out[12] = w.z;
  return out;
}

export function unpackState(y: Float64Array): { position: Vec3; orientation: Quat; velBody: Vec3; omegaBody: Vec3 } {
  return {
    position: { x: y[0] as number, y: y[1] as number, z: y[2] as number },
    orientation: { w: y[3] as number, x: y[4] as number, y: y[5] as number, z: y[6] as number },
    velBody: { x: y[7] as number, y: y[8] as number, z: y[9] as number },
    omegaBody: { x: y[10] as number, y: y[11] as number, z: y[12] as number },
  };
}

/** Evaluate the fluid wrench (body frame, about the CG) for the state y at time t. */
export type WrenchFn = (y: Float64Array, t: number, out: Wrench) => void;

/**
 * State derivative dy/dt for the given body and external wrench. Writes into `out`.
 */
export function rigidBodyDerivative(y: Float64Array, wrench: Readonly<Wrench>, body: Readonly<BodyProperties>, out: Float64Array): void {
  const qw = y[3] as number;
  const qx = y[4] as number;
  const qy = y[5] as number;
  const qz = y[6] as number;
  const u = y[7] as number;
  const v = y[8] as number;
  const w = y[9] as number;
  const p = y[10] as number;
  const q = y[11] as number;
  const r = y[12] as number;
  // position rate: R(q) ν₁
  const r00 = 1 - 2 * (qy * qy + qz * qz);
  const r01 = 2 * (qx * qy - qw * qz);
  const r02 = 2 * (qx * qz + qw * qy);
  const r10 = 2 * (qx * qy + qw * qz);
  const r11 = 1 - 2 * (qx * qx + qz * qz);
  const r12 = 2 * (qy * qz - qw * qx);
  const r20 = 2 * (qx * qz - qw * qy);
  const r21 = 2 * (qy * qz + qw * qx);
  const r22 = 1 - 2 * (qx * qx + qy * qy);
  out[0] = r00 * u + r01 * v + r02 * w;
  out[1] = r10 * u + r11 * v + r12 * w;
  out[2] = r20 * u + r21 * v + r22 * w;
  // attitude rate: q̇ = ½ q ⊗ (0, ω)
  out[3] = 0.5 * (-qx * p - qy * q - qz * r);
  out[4] = 0.5 * (qw * p + qy * r - qz * q);
  out[5] = 0.5 * (qw * q + qz * p - qx * r);
  out[6] = 0.5 * (qw * r + qx * q - qy * p);
  // gravity in the body frame: Rᵀ (0, 0, g)
  const m = body.mass;
  const gx = r20 * body.g;
  const gy = r21 * body.g;
  const gz = r22 * body.g;
  const A = body.addedMass;
  const I = body.inertia;
  const mask = body.mask;
  // translational: (m + A) ν̇₁ = F + m g_b − m (ω × ν₁)
  const fx = wrench.force.x + m * gx - m * (q * w - r * v);
  const fy = wrench.force.y + m * gy - m * (r * u - p * w);
  const fz = wrench.force.z + m * gz - m * (p * v - q * u);
  out[7] = mask.surge ? fx / (m + A.surge) : 0;
  out[8] = mask.sway ? fy / (m + A.sway) : 0;
  out[9] = mask.heave ? fz / (m + A.heave) : 0;
  // rotational: (I + A) ω̇ = M − ω × (I ω)
  const Ix = I.x;
  const Iy = I.y;
  const Iz = I.z;
  const mx = wrench.moment.x - (q * Iz * r - r * Iy * q);
  const my = wrench.moment.y - (r * Ix * p - p * Iz * r);
  const mz = wrench.moment.z - (p * Iy * q - q * Ix * p);
  out[10] = mask.roll ? mx / (Ix + A.roll) : 0;
  out[11] = mask.pitch ? my / (Iy + A.pitch) : 0;
  out[12] = mask.yaw ? mz / (Iz + A.yaw) : 0;
}

/** Zero the velocities of locked DOFs and renormalise the attitude. */
export function enforceConstraints(y: Float64Array, mask: DofMask): void {
  if (!mask.surge) y[7] = 0;
  if (!mask.sway) y[8] = 0;
  if (!mask.heave) y[9] = 0;
  if (!mask.roll) y[10] = 0;
  if (!mask.pitch) y[11] = 0;
  if (!mask.yaw) y[12] = 0;
  const n = Math.hypot(y[3] as number, y[4] as number, y[5] as number, y[6] as number) || 1;
  y[3] = (y[3] as number) / n;
  y[4] = (y[4] as number) / n;
  y[5] = (y[5] as number) / n;
  y[6] = (y[6] as number) / n;
}
