import type { Dof6, ForceContext, ForceModel, Wrench } from '../core/types';
import { rotate, rotateInverse } from '../math/quat';
import type { Vec3 } from '../math/vec3';

/**
 * Pressure loads from the shared hydrostatic integration (computed once per evaluation by the
 * simulation), rotated into the body frame.
 */
export function hydrostaticForce(): ForceModel {
  const f: Vec3 = { x: 0, y: 0, z: 0 };
  const mo: Vec3 = { x: 0, y: 0, z: 0 };
  return {
    name: 'hydrostatics',
    apply(ctx, out) {
      rotateInverse(ctx.orientation, ctx.hydro.force, f);
      rotateInverse(ctx.orientation, ctx.hydro.moment, mo);
      out.force.x += f.x;
      out.force.y += f.y;
      out.force.z += f.z;
      out.moment.x += mo.x;
      out.moment.y += mo.y;
      out.moment.z += mo.z;
      ctx.diag['hydro.volume'] = ctx.hydro.volume;
      ctx.diag['hydro.wettedArea'] = ctx.hydro.wettedArea;
    },
  };
}

/** Applies the wetted-panel cross-flow drag computed by the hydrostatics pass (see crossflowCd). */
export function panelCrossflow(): ForceModel {
  const f: Vec3 = { x: 0, y: 0, z: 0 };
  const mo: Vec3 = { x: 0, y: 0, z: 0 };
  return {
    name: 'panelCrossflow',
    apply(ctx, out) {
      rotateInverse(ctx.orientation, ctx.hydro.crossflowForce, f);
      rotateInverse(ctx.orientation, ctx.hydro.crossflowMoment, mo);
      out.force.x += f.x;
      out.force.y += f.y;
      out.force.z += f.z;
      out.moment.x += mo.x;
      out.moment.y += mo.y;
      out.moment.z += mo.z;
      ctx.diag['crossflow.fy'] = f.y;
    },
  };
}

export interface DampingOptions {
  /** Linear coefficients (N per m/s, N·m per rad/s). */
  linear?: Partial<Dof6>;
  /** Quadratic coefficients (N per (m/s)², N·m per (rad/s)²). */
  quadratic?: Partial<Dof6>;
  /** Scale the damping by how much of the hull is in the water (default true). */
  scaleWithImmersion?: boolean;
}

/** Linear + quadratic damping on the velocity relative to the water. */
export function viscousDamping(opts: DampingOptions): ForceModel {
  const l = opts.linear ?? {};
  const q = opts.quadratic ?? {};
  const g = (d: Partial<Dof6>, k: keyof Dof6): number => d[k] ?? 0;
  const L = { surge: g(l, 'surge'), sway: g(l, 'sway'), heave: g(l, 'heave'), roll: g(l, 'roll'), pitch: g(l, 'pitch'), yaw: g(l, 'yaw') };
  const Q = { surge: g(q, 'surge'), sway: g(q, 'sway'), heave: g(q, 'heave'), roll: g(q, 'roll'), pitch: g(q, 'pitch'), yaw: g(q, 'yaw') };
  const scale = opts.scaleWithImmersion ?? true;
  const vw: Vec3 = { x: 0, y: 0, z: 0 };
  const fb: Vec3 = { x: 0, y: 0, z: 0 };
  return {
    name: 'damping',
    apply(ctx, out) {
      const k = scale ? immersionFactor(ctx) : 1;
      if (k === 0) return;
      const v = ctx.relVelBody;
      const w = ctx.omegaBody;
      out.force.x -= k * (L.surge * v.x + Q.surge * v.x * Math.abs(v.x));
      out.force.y -= k * (L.sway * v.y + Q.sway * v.y * Math.abs(v.y));
      // heave damping acts on the vertical (world) velocity: a trimmed hull running steadily has a
      // body-z velocity component that must not be damped
      rotate(ctx.orientation, v, vw);
      vw.x = 0;
      vw.y = 0;
      vw.z = -k * (L.heave * vw.z + Q.heave * vw.z * Math.abs(vw.z));
      rotateInverse(ctx.orientation, vw, fb);
      out.force.x += fb.x;
      out.force.y += fb.y;
      out.force.z += fb.z;
      out.moment.x -= k * (L.roll * w.x + Q.roll * w.x * Math.abs(w.x));
      out.moment.y -= k * (L.pitch * w.y + Q.pitch * w.y * Math.abs(w.y));
      out.moment.z -= k * (L.yaw * w.z + Q.yaw * w.z * Math.abs(w.z));
    },
  };
}

/** 0 when the hull is out of the water, 1 once it displaces a meaningful volume. */
export function immersionFactor(ctx: ForceContext): number {
  const f = ctx.hydro.submergedVertexFraction;
  return f <= 0 ? 0 : Math.min(1, f / 0.05);
}

export interface WindageOptions {
  frontalArea: number;
  lateralArea: number;
  /** Centroid of the lateral area relative to the CG, body frame (z negative = above the CG). */
  centroid: Readonly<Vec3>;
  cdFrontal?: number;
  cdLateral?: number;
}

/**
 * Wind load on the above-water profile: drag on the frontal and lateral projections of the
 * apparent wind, applied at the lateral-area centroid. A deliberately simple stand-in for the
 * Isherwood (1972) regression.
 */
export function windage(o: WindageOptions): ForceModel {
  const cdF = o.cdFrontal ?? 0.7;
  const cdL = o.cdLateral ?? 0.9;
  return {
    name: 'windage',
    apply(ctx, out) {
      const a = ctx.windRelBody;
      const V = Math.hypot(a.x, a.y);
      if (V < 1e-6) return;
      const q = 0.5 * ctx.env.rhoAir * V;
      const fx = q * cdF * o.frontalArea * a.x;
      const fy = q * cdL * o.lateralArea * a.y;
      out.force.x += fx;
      out.force.y += fy;
      const c = o.centroid;
      out.moment.x += -c.z * fy;
      out.moment.y += c.z * fx;
      out.moment.z += c.x * fy;
      ctx.diag['wind.apparentSpeed'] = V;
      ctx.diag['wind.fy'] = fy;
    },
  };
}

export interface ThrusterOptions {
  maxForce: number;
  /** Body-frame position relative to the CG. */
  position: Readonly<Vec3>;
  /** Forward speed (m/s) above which the thruster loses its effect. */
  cutoffSpeed: number;
}

/** Tunnel bow thruster: lateral force that fades with forward speed. */
export function bowThruster(o: ThrusterOptions): ForceModel {
  return {
    name: 'bowThruster',
    apply(ctx, out) {
      const c = ctx.actuators.thruster;
      if (c === 0) return;
      const fade = Math.max(0, 1 - Math.abs(ctx.relVelBody.x) / o.cutoffSpeed);
      const fy = c * o.maxForce * fade * immersionFactor(ctx);
      out.force.y += fy;
      out.moment.x += -o.position.z * fy;
      out.moment.z += o.position.x * fy;
      ctx.diag['thruster.force'] = fy;
    },
  };
}

/**
 * Coriolis and centripetal terms of the added mass (Fossen C_A(ν)ν for diagonal added mass).
 * `munk` toggles the destabilising Munk moment N = −(A_sway − A_surge) u v, which captive-test
 * hull derivatives (MMG) already contain.
 */
export function addedMassCoriolis(added: Readonly<Dof6>, opts: { munk?: boolean } = {}): ForceModel {
  const munk = opts.munk ?? true;
  return {
    name: 'addedMassCoriolis',
    apply(ctx, out: Wrench) {
      const k = immersionFactor(ctx);
      if (k === 0) return;
      const { x: u, y: v, z: w } = ctx.relVelBody;
      const { x: p, y: q, z: r } = ctx.omegaBody;
      const A = added;
      // −C_A(ν) ν with diagonal M_A (Fossen 2021, Eq. 6.43)
      out.force.x += k * (A.sway * v * r - A.heave * w * q);
      out.force.y += k * (A.heave * w * p - A.surge * u * r);
      out.force.z += k * (A.surge * u * q - A.sway * v * p);
      out.moment.x += k * ((A.sway - A.heave) * v * w + (A.pitch - A.yaw) * q * r);
      out.moment.y += k * ((A.heave - A.surge) * w * u + (A.yaw - A.roll) * r * p);
      if (munk) out.moment.z += k * ((A.surge - A.sway) * u * v + (A.roll - A.pitch) * p * q);
      else out.moment.z += k * ((A.roll - A.pitch) * p * q);
    },
  };
}
