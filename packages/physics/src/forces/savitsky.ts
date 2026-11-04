import type { ForceContext, ForceModel } from '../core/types';
import { rotate } from '../math/quat';
import { bisect, ittc57Cf, smoothstep } from '../math/scalar';
import type { Vec3 } from '../math/vec3';

/**
 * Savitsky (1964) planing-surface equations. Angles in the empirical formulas are in degrees,
 * exactly as published; everything else is SI.
 */
const RAD = Math.PI / 180;

/** Beam Froude number Cv = V / √(g b). */
export const savitskyCv = (V: number, beam: number, g: number): number => V / Math.sqrt(g * beam);

/** Flat-plate lift coefficient, dynamic + hydrostatic terms. */
export const savitskyCL0 = (tauDeg: number, lambda: number, cv: number): number =>
  tauDeg ** 1.1 * (0.012 * Math.sqrt(lambda) + (0.0055 * lambda ** 2.5) / (cv * cv));

/** Dynamic part of the flat-plate lift coefficient (the 1/Cv² hydrostatic term removed). */
export const savitskyCL0Dynamic = (tauDeg: number, lambda: number): number => tauDeg ** 1.1 * 0.012 * Math.sqrt(lambda);

/** Deadrise correction: C_Lβ = C_L0 − 0.0065 β C_L0^0.6. */
export const savitskyCLBeta = (cl0: number, betaDeg: number): number => Math.max(0, cl0 - 0.0065 * betaDeg * Math.max(cl0, 0) ** 0.6);

/** Centre of pressure from the transom along the keel, as a fraction of the mean wetted length. */
export const savitskyCpFraction = (cv: number, lambda: number): number => 0.75 - 1 / ((5.21 * cv * cv) / (lambda * lambda) + 2.39);

/** Mean bottom velocity over the forward speed. */
export function savitskyMeanVelocityRatio(tauDeg: number, lambda: number, betaDeg: number): number {
  const d = 0.012 * Math.sqrt(lambda) * tauDeg ** 1.1;
  const num = d - 0.0065 * betaDeg * Math.max(d, 0) ** 0.6;
  const r = 1 - num / (lambda * Math.cos(tauDeg * RAD));
  return Math.sqrt(Math.max(r, 0));
}

export interface SavitskyPoint {
  tauDeg: number;
  lambda: number;
  cv: number;
  cl0: number;
  clBeta: number;
  lift: number;
  meanVelocity: number;
  cf: number;
  friction: number;
  pressureDrag: number;
  /** Total resistance, Savitsky's D = Δ tan τ + D_f / cos τ (N). */
  resistance: number;
  /** Centre of pressure from the transom (m). */
  cp: number;
}

export interface SavitskyHull {
  beam: number;
  deadriseDeg: number;
  /** Weight Δ (N). */
  weight: number;
  /** Longitudinal centre of gravity from the transom (m). */
  lcg: number;
  deltaCf?: number;
}

/** Evaluate the Savitsky components at a given speed, trim and wetted-length ratio. */
export function savitskyEvaluate(
  hull: Omit<SavitskyHull, 'lcg'>,
  V: number,
  tauDeg: number,
  lambda: number,
  rho: number,
  nu: number,
  g: number,
): SavitskyPoint {
  const b = hull.beam;
  const cv = savitskyCv(V, b, g);
  const cl0 = savitskyCL0(tauDeg, lambda, cv);
  const clBeta = savitskyCLBeta(cl0, hull.deadriseDeg);
  const lift = 0.5 * rho * V * V * b * b * clBeta;
  const vm = V * savitskyMeanVelocityRatio(tauDeg, lambda, hull.deadriseDeg);
  const re = (vm * lambda * b) / nu;
  const cf = ittc57Cf(re) + (hull.deltaCf ?? 0);
  const S = (lambda * b * b) / Math.cos(hull.deadriseDeg * RAD);
  const friction = 0.5 * rho * vm * vm * S * cf;
  const tau = tauDeg * RAD;
  const pressureDrag = lift * Math.tan(tau);
  return {
    tauDeg,
    lambda,
    cv,
    cl0,
    clBeta,
    lift,
    meanVelocity: vm,
    cf,
    friction,
    pressureDrag,
    resistance: hull.weight * Math.tan(tau) + friction / Math.cos(tau),
    cp: savitskyCpFraction(cv, lambda) * lambda * b,
  };
}

/**
 * Savitsky's simplified equilibrium (all forces through the CG): find the trim τ and wetted
 * length λ where lift equals weight and the centre of pressure lies at the LCG.
 */
export function savitskyEquilibrium(hull: SavitskyHull, V: number, rho: number, nu: number, g: number): SavitskyPoint {
  const lambdaFor = (tauDeg: number): number => {
    const f = (lam: number): number => savitskyEvaluate(hull, V, tauDeg, lam, rho, nu, g).lift - hull.weight;
    if (f(20) < 0) return 20;
    return bisect(f, 1e-4, 20, 1e-9);
  };
  const cpErr = (tauDeg: number): number => {
    const lam = lambdaFor(tauDeg);
    return savitskyEvaluate(hull, V, tauDeg, lam, rho, nu, g).cp - hull.lcg;
  };
  // cp moves forward as trim drops (longer wetted length): bracket on [0.5°, 15°]
  let lo = 0.5;
  let hi = 15;
  if (Math.sign(cpErr(lo)) === Math.sign(cpErr(hi))) {
    // outside Savitsky's range: return the end closest to balance
    const tau = Math.abs(cpErr(lo)) < Math.abs(cpErr(hi)) ? lo : hi;
    return savitskyEvaluate(hull, V, tau, lambdaFor(tau), rho, nu, g);
  }
  const tau = bisect(cpErr, lo, hi, 1e-7);
  lo = tau;
  hi = tau;
  return savitskyEvaluate(hull, V, tau, lambdaFor(tau), rho, nu, g);
}

export interface PlaningModelOptions {
  /** Chine beam (m). */
  beam: number;
  /** Deadrise of the prismatic bottom (rad). */
  deadrise: number;
  /** Keel point at the transom, body frame relative to the CG (m). */
  transomKeel: Readonly<Vec3>;
  /** Maximum wetted keel length (m): the prismatic part plus the forefoot. */
  maxWettedLength: number;
  deltaCf?: number;
  /** Cv below which dynamic lift fades out (smoothly between lo and hi). */
  liftFade?: { lo: number; hi: number };
  /**
   * Forefoot geometry ahead of the prismatic bottom (see geometry/planingHull): prismatic length
   * from the transom, total keel length, keel rise at the stem and deadrise at the stem. When set,
   * the wetted forefoot adds its own Savitsky lift at its larger local buttock angle; this is what
   * lifts the bow over the hump at low trim.
   */
  forefoot?: { prismaticLength: number; length: number; rise: number; deadriseAtStem: number };
}

/**
 * Time-domain Savitsky model. Each evaluation reads the instantaneous trim relative to the local
 * water surface, the transom keel depth and the flow angle at the bottom, derives the wetted length
 * λ from geometry, and applies:
 * - dynamic lift (the 0.012 λ^½ τ^1.1 term with the deadrise correction) normal to the bottom at
 *   Savitsky's centre of pressure — buoyancy already comes from the hull mesh, so the hydrostatic
 *   term is left out;
 * - frictional drag on the wetted bottom at the mean bottom velocity (ITTC-57 + ΔCf).
 * Running trim, sinkage and resistance then emerge from the 6-DOF dynamics.
 */
export function savitskyPlaning(o: PlaningModelOptions): ForceModel {
  const b = o.beam;
  const betaDeg = o.deadrise / RAD;
  const tanBeta = Math.tan(o.deadrise);
  const fade = o.liftFade ?? { lo: 0.6, hi: 1.6 };
  const keelWorld: Vec3 = { x: 0, y: 0, z: 0 };
  const fwdWorld: Vec3 = { x: 0, y: 0, z: 0 };
  return {
    name: 'savitsky',
    apply(ctx: ForceContext, out) {
      const { rho, nu, g } = ctx.env;
      const q = ctx.orientation;
      const p = ctx.position;
      const tk = o.transomKeel;
      // deepest point of the transom bottom: the keel, or the low chine when heeled hard
      const chineUp = (b / 2) * tanBeta;
      let keelDepth = -Infinity;
      for (let side = -1; side <= 1; side++) {
        const pt = side === 0 ? tk : { x: tk.x, y: tk.y + side * (b / 2), z: tk.z - chineUp };
        rotate(q, pt, keelWorld);
        const d = p.z + keelWorld.z + ctx.env.elevation(p.x + keelWorld.x, p.y + keelWorld.y, ctx.t);
        if (d > keelDepth) keelDepth = d;
      }
      rotate(q, tk, keelWorld);
      const kx = p.x + keelWorld.x;
      const ky = p.y + keelWorld.y;
      const dg = ctx.diag;
      dg['planing.keelDepth'] = keelDepth;
      if (keelDepth <= 0) {
        dg['planing.lift'] = 0;
        dg['planing.lambda'] = 0;
        return;
      }
      // heel flattens the low side of the V: plane on an effective deadrise of β − |φ|
      const heel = Math.abs(ctx.euler.roll);
      const betaEffDeg = Math.max(betaDeg - heel / RAD, 5);
      // geometric trim relative to the local surface: keel slope minus surface slope along the keel
      const probe = 0.5 * o.maxWettedLength;
      rotate(q, { x: tk.x + probe, y: tk.y, z: tk.z }, fwdWorld);
      const fx = p.x + fwdWorld.x;
      const fy = p.y + fwdWorld.y;
      const keelRise = keelWorld.z - fwdWorld.z; // how much higher the forward probe is (NED z down)
      const surfRise = ctx.env.elevation(fx, fy, ctx.t) - ctx.env.elevation(kx, ky, ctx.t);
      const horiz = Math.max(Math.hypot(fx - kx, fy - ky), 1e-6);
      const tauGeo = Math.atan2(keelRise - surfRise, horiz);
      // wetted keel and chine lengths
      let Lk = tauGeo > 0.3 * RAD ? keelDepth / Math.sin(tauGeo) : o.maxWettedLength;
      Lk = Math.min(Lk, o.forefoot ? o.forefoot.prismaticLength : o.maxWettedLength);
      const Lc = Math.max(0, Lk - (b * tanBeta) / (Math.PI * Math.tan(Math.max(tauGeo, 0.3 * RAD))));
      const lambda = Math.max((Lk + Lc) / (2 * b), 0.05);
      // flow at the centre of pressure, body frame
      const rel = ctx.relVelBody;
      const w = ctx.omegaBody;
      const V = Math.max(rel.x, 0);
      const cv = savitskyCv(V, b, g);
      const lp = Math.min(Math.max(savitskyCpFraction(Math.max(cv, 0.5), lambda), 0.1), 0.9) * lambda * b;
      const cpx = tk.x + lp;
      const cpz = tk.z;
      // vertical velocity of the bottom at the CP relative to the water (sinking = +): heave and
      // pitch motion change the angle of attack, which is the dominant planing damping
      const pcp = { x: rel.x + w.y * cpz, y: rel.y + w.z * cpx - w.x * cpz, z: rel.z - w.y * cpx };
      rotate(q, pcp, fwdWorld);
      const alpha = tauGeo + Math.atan2(fwdWorld.z, Math.max(V, 1e-3));
      const tauDeg = Math.min(Math.max(alpha / RAD, 0), 10);
      const ramp = smoothstep(fade.lo, fade.hi, cv);
      // split the bottom into its two halves: heel flattens the low half (more lift) and steepens
      // the high half (less lift), which gives the planing hull its dynamic roll stiffness
      const cl0 = savitskyCL0Dynamic(tauDeg, lambda);
      const phiDeg = ctx.euler.roll / RAD;
      const clStbd = savitskyCLBeta(cl0, Math.max(betaDeg - phiDeg, 3));
      const clPort = savitskyCLBeta(cl0, Math.max(betaDeg + phiDeg, 3));
      const qb = ramp * 0.5 * rho * V * V * b * b;
      const lift = qb * 0.5 * (clStbd + clPort);
      // starboard half at y = +b/4: more lift there rolls the hull to port (negative K)
      out.moment.x += -(qb * 0.5 * (clStbd - clPort)) * (b / 4);
      void betaEffDeg;
      const normal = lift / Math.cos(tauDeg * RAD);
      // normal force acts up the body z axis at the CP
      out.force.z -= normal;
      // moment r × F with r = (cpx, 0, cpz), F = (0, 0, −N)
      out.moment.y += cpx * normal;
      // sideslip loads the leading side of the V bottom harder, banking the hull into the turn:
      // K ≈ −L (b/4) (v/U) / tan β (a first-order asymmetric-deadrise estimate)
      if (V > 1) {
        const slip = Math.max(-0.15, Math.min(0.15, rel.y / V));
        out.moment.x += -lift * (b / 4) * (slip / tanBeta);
      }
      // forefoot: wetted part of the rising keel ahead of the prismatic bottom
      let bowLift = 0;
      const ff = o.forefoot;
      if (ff && V > 0.5) {
        const P = ff.prismaticLength;
        const span = ff.length - P;
        const st = Math.sin(tauGeo);
        const ct = Math.cos(tauGeo);
        const depthAt = (x: number): number => {
          const t = (x - P) / span;
          return keelDepth - x * st - ff.rise * t * t * ct;
        };
        if (depthAt(P) > 0) {
          const xEnd = depthAt(ff.length) > 0 ? ff.length : bisect(depthAt, P, ff.length, 1e-4, 40);
          const len = xEnd - P;
          if (len > 1e-3) {
            const tEnd = len / span;
            const slope = Math.atan2(ff.rise * tEnd * tEnd, len);
            const bb = b * Math.max(0.2, 1 - (tEnd * tEnd) / 6);
            const lamB = Math.max(len / bb, 0.05);
            const tauB = Math.min(Math.max((alpha + slope) / RAD, 0), 8);
            const drB = (o.deadrise + (ff.deadriseAtStem - o.deadrise) * 0.5 * tEnd) / RAD;
            const clB = savitskyCLBeta(savitskyCL0Dynamic(tauB, lamB), drB);
            bowLift = ramp * 0.5 * rho * V * V * bb * bb * clB;
            const xb = tk.x + P + 0.5 * len;
            const nB = bowLift / Math.cos(tauB * RAD);
            out.force.z -= nB;
            out.moment.y += xb * nB;
          }
        }
      }
      dg['planing.bowLift'] = bowLift;
      // added-mass flux damping of the wetted bottom (Wagner-type, c = π/4 ρ V b²), spread over the
      // wetted length: damps heave and pitch in proportion to speed, as in Zarnick-type strip models
      if (V > 0.5) {
        const lw = lambda * b;
        const ch = (Math.PI / 4) * rho * V * b * b * Math.min(1, lambda);
        const xc = tk.x + 0.5 * lw;
        // vertical (world) velocity of the wetted-area centre: steady planing has none, so only
        // heaving and pitching motion is damped
        const pv = { x: rel.x + w.y * tk.z, y: rel.y + w.z * xc - w.x * tk.z, z: rel.z - w.y * xc };
        rotate(q, pv, fwdWorld);
        const wc = fwdWorld.z;
        out.moment.y += xc * ch * wc - ch * ((lw * lw) / 12) * w.y;
      }
      // friction on the wetted bottom
      let friction = 0;
      if (V > 0.05) {
        const vm = V * savitskyMeanVelocityRatio(tauDeg, lambda, betaDeg);
        const re = (vm * lambda * b) / nu;
        const S = (lambda * b * b) / Math.cos(o.deadrise);
        friction = 0.5 * rho * vm * vm * S * (ittc57Cf(re) + (o.deltaCf ?? 0.0004));
        const fxz = tk.x + 0.5 * lambda * b;
        out.force.x -= friction;
        // F = (−Df, 0, 0) at (fxz, 0, cpz): M_y = z F_x − x F_z = −cpz Df
        out.moment.y += -cpz * friction;
        void fxz;
      }
      dg['planing.lift'] = lift;
      dg['planing.lambda'] = lambda;
      dg['planing.trimDeg'] = tauGeo / RAD;
      dg['planing.cv'] = cv;
      dg['planing.friction'] = friction;
    },
  };
}

/**
 * Pre-planing (hump) residuary resistance. Mesh hydrostatics carries no wave-making resistance
 * and Savitsky's equations are only valid on the plane, so this adds a smooth hump in R/Δ around
 * volumetric Froude number 1–2. Calibrated to typical planing-hull resistance curves, not a
 * published regression; replace with Savitsky & Brown (1976) when that lands.
 */
export function preplaningResistance(o: {
  weight: number;
  volume: number;
  peak?: number;
  center?: number;
  width?: number;
  /** Pitch stiffness (N·m/rad) and peak hump trim (rad): the bow-up moment of the bow wave. */
  pitchStiffness?: number;
  humpTrim?: number;
}): ForceModel {
  const peak = o.peak ?? 0.07;
  const humpMoment = (o.pitchStiffness ?? 0) * (o.humpTrim ?? 0);
  const center = o.center ?? 1.3;
  const width = o.width ?? 0.55;
  const lv = Math.cbrt(o.volume);
  return {
    name: 'preplaning',
    apply(ctx, out) {
      const u = ctx.relVelBody.x;
      const fnv = Math.abs(u) / Math.sqrt(ctx.env.g * lv);
      const shape = Math.exp(-(((fnv - center) / width) ** 2));
      // low-speed rise ∝ Fn⁴ blends into the hump
      const r = o.weight * peak * shape * smoothstep(0, 0.8, fnv);
      out.force.x -= Math.sign(u) * r;
      // the bow wave and stern trough of the hump trim the bow up; fades out once on the plane
      const trimShape = Math.exp(-(((fnv - center) / (1.3 * width)) ** 2)) * smoothstep(0.2, 1.0, fnv);
      out.moment.y += u > 0 ? humpMoment * trimShape : 0;
      ctx.diag['planing.hump'] = r;
    },
  };
}
