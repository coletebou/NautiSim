import type { ForceContext, ForceModel } from '../core/types';
import { rotate } from '../math/quat';
import { bisect } from '../math/scalar';
import type { Vec3 } from '../math/vec3';

/**
 * Open-water propeller characteristics, a smooth fit with the shape of the Wageningen B-series:
 * KT = KT0 (1 − J/J0)(1 + 0.4 J/J0), KQ likewise, with J0 ≈ 1.08 P/D. Peak open-water efficiency
 * comes out near 0.65–0.7, typical of small-craft three-blade screws.
 */
export interface PropellerCurve {
  diameter: number;
  pitchRatio: number;
}

export function propellerKT(p: PropellerCurve, J: number): number {
  const J0 = 1.08 * p.pitchRatio;
  const kt0 = 0.33 * p.pitchRatio + 0.1;
  const x = J / J0;
  return kt0 * (1 - x) * (1 + 0.4 * x);
}

export function propellerKQ(p: PropellerCurve, J: number): number {
  const J0 = 1.08 * p.pitchRatio * 1.1;
  const kt0 = 0.33 * p.pitchRatio + 0.1;
  const kq0 = 0.15 * p.pitchRatio * kt0;
  const x = J / J0;
  return kq0 * (1 - x) * (1 + 0.4 * x);
}

export interface OutboardOptions extends PropellerCurve {
  /** Propeller centre, body frame relative to the CG (m). */
  position: Readonly<Vec3>;
  /** Engine rated power (W) — the most the propeller can absorb. */
  ratedPower: number;
  /** Effective wake fraction (small for outboards). */
  wake?: number;
  /** Thrust line angle up from the body x axis (rad), i.e. outboard trim. */
  thrustAngle?: number;
  /** Astern thrust efficiency relative to ahead. */
  asternFactor?: number;
  /** Lower-unit (leg) lateral area (m²) and lift slope (per rad). */
  legArea?: number;
  legLiftSlope?: number;
  /** Leg centre, body frame (m); defaults to slightly above the propeller. */
  legPosition?: Readonly<Vec3>;
}

/**
 * Steerable outboard or sterndrive: the commanded rps (from the throttle) is capped by the
 * power the engine can deliver at the current advance ratio, thrust is vectored by the steering
 * angle, and the lower unit adds a rudder-like side force. Positive steering turns to starboard
 * (the thrust pushes the stern to port).
 */
export function outboardDrive(o: OutboardOptions): ForceModel {
  const D = o.diameter;
  const wake = o.wake ?? 0.05;
  const thrustAngle = o.thrustAngle ?? 0;
  const astern = o.asternFactor ?? 0.55;
  const legArea = o.legArea ?? 0;
  const legSlope = o.legLiftSlope ?? 3.0;
  const leg = o.legPosition ?? { x: o.position.x, y: o.position.y, z: o.position.z - 0.15 };
  const propWorld: Vec3 = { x: 0, y: 0, z: 0 };
  return {
    name: 'outboard',
    apply(ctx: ForceContext, out) {
      const { rho } = ctx.env;
      // only works with the propeller in the water
      rotate(ctx.orientation, o.position, propWorld);
      const px = ctx.position.x + propWorld.x;
      const py = ctx.position.y + propWorld.y;
      const depth = ctx.position.z + propWorld.z + ctx.env.elevation(px, py, ctx.t);
      const submergence = Math.max(0, Math.min(1, (depth + 0.5 * D) / D));
      const nCmd = ctx.actuators.rps;
      const delta = ctx.actuators.rudder;
      const u = ctx.relVelBody.x;
      const Va = Math.max(u, 0) * (1 - wake);
      let n = 0;
      let thrust = 0;
      let power = 0;
      if (Math.abs(nCmd) > 1e-3 && submergence > 0) {
        const sign = Math.sign(nCmd);
        const powerAt = (nn: number): number => {
          const J = nn > 1e-6 ? Va / (nn * D) : 0;
          return 2 * Math.PI * nn * rho * nn * nn * D ** 5 * Math.max(propellerKQ(o, J), 1e-4);
        };
        const nAbs = Math.abs(nCmd);
        const available = o.ratedPower * Math.max(Math.abs(ctx.actuators.throttle), 0.05);
        n = powerAt(nAbs) <= available ? nAbs : bisect((nn) => powerAt(nn) - available, 1e-3, nAbs, 1e-6, 40);
        const J = sign > 0 ? Va / (n * D) : 0;
        const kt = Math.max(propellerKT(o, J), sign > 0 ? -0.05 : 0);
        thrust = sign * (sign > 0 ? 1 : astern) * rho * n * n * D ** 4 * kt * submergence;
        power = powerAt(n);
        n *= sign;
      }
      const cd = Math.cos(delta);
      const sd = Math.sin(delta);
      const ct = Math.cos(thrustAngle);
      const st = Math.sin(thrustAngle);
      const Fx = thrust * cd * ct;
      const Fy = -thrust * sd * ct;
      const Fz = -thrust * st;
      const r = o.position;
      out.force.x += Fx;
      out.force.y += Fy;
      out.force.z += Fz;
      out.moment.x += r.y * Fz - r.z * Fy;
      out.moment.y += r.z * Fx - r.x * Fz;
      out.moment.z += r.x * Fy - r.y * Fx;
      // lower unit as a small rudder: normal force against the normal velocity of the leg
      if (legArea > 0 && submergence > 0) {
        const w = ctx.omegaBody;
        const ul = ctx.relVelBody.x - w.z * leg.y;
        const vl = ctx.relVelBody.y + w.z * leg.x - w.x * leg.z;
        const U = Math.hypot(ul, vl);
        if (U > 0.05) {
          const vn = ul * sd + vl * cd;
          const cap = Math.sin(20 * (Math.PI / 180)) * U;
          const vnC = Math.max(-cap, Math.min(cap, vn));
          const fn = -0.5 * rho * legArea * legSlope * U * vnC * submergence;
          const lx = fn * sd;
          const ly = fn * cd;
          out.force.x += lx;
          out.force.y += ly;
          out.moment.x += -leg.z * ly;
          out.moment.y += leg.z * lx;
          out.moment.z += leg.x * ly - leg.y * lx;
          ctx.diag['outboard.legForce'] = ly;
        }
      }
      ctx.diag['engine.rps'] = n;
      ctx.diag['engine.power'] = power;
      ctx.diag['outboard.thrust'] = thrust;
    },
  };
}

export interface LateralHullOptions {
  /** Keel point at the transom relative to the CG, body frame (m). */
  transomKeel: Readonly<Vec3>;
  /** Hull length over which the keel can be wetted (m). */
  length: number;
  /** Cross-flow drag coefficient of the V sections. */
  crossflowCd?: number;
  /** Strips along the wetted length. */
  strips?: number;
}

/**
 * Sway and yaw loads on a planing or semi-displacement hull: slender-body lift of the immersed keel
 * (Jones, Y = −π/2 ρ T² U v, distributed with the local draft) plus quadratic cross-flow drag on
 * each strip (Fossen's strip-theory cross-flow model). Both act at half the local draft.
 */
export function lateralHull(o: LateralHullOptions): ForceModel {
  const cd = o.crossflowCd ?? 1.0;
  const nStrips = o.strips ?? 8;
  const tk = o.transomKeel;
  return {
    name: 'lateralHull',
    apply(ctx, out) {
      const keelDepth = ctx.diag['planing.keelDepth'] ?? ctx.hydro.maxDepth;
      if (!(keelDepth > 0)) return;
      const { rho } = ctx.env;
      const pitch = ctx.euler.pitch;
      const tau = Math.max(pitch, 0.5 * (Math.PI / 180));
      const wetted = Math.min(o.length, keelDepth / Math.sin(tau));
      const v = ctx.relVelBody.y;
      const u = ctx.relVelBody.x;
      const r = ctx.omegaBody.z;
      const p = ctx.omegaBody.x;
      const U = Math.hypot(u, v);
      const dx = wetted / nStrips;
      let Fy = 0;
      let Mz = 0;
      let Mx = 0;
      // the slender-body lift coefficient per unit length: d/dx(π/2 ρ T(x)² ) U v → concentrated where
      // the draft changes; we spread it as π ρ T dT/dx, which integrates to the Jones total
      for (let i = 0; i < nStrips; i++) {
        const s = (i + 0.5) * dx; // from the transom
        const T = Math.max(0, keelDepth - s * Math.sin(tau));
        if (T <= 0) continue;
        const x = tk.x + s;
        const zc = tk.z - 0.5 * T;
        const vl = v + r * x - p * zc;
        const drag = cd > 0 ? -0.5 * rho * cd * T * dx * Math.abs(vl) * vl : 0;
        const lift = -(Math.PI / 2) * rho * T * T * (dx / wetted) * U * vl;
        const f = drag + lift;
        Fy += f;
        Mz += x * f;
        Mx += -zc * f;
      }
      out.force.y += Fy;
      out.moment.z += Mz;
      out.moment.x += Mx;
      ctx.diag['lateral.fy'] = Fy;
    },
  };
}
