import type { ForceModel } from '../core/types';
import { schoenherrCf } from '../math/scalar';

/**
 * MMG standard method (Yasukawa & Yoshimura 2015): hull, propeller and rudder forces in the
 * horizontal plane, written about midship and transferred to the CG. Non-dimensional coefficients
 * are by ½ρLd U² (forces) and ½ρL²d U² (moments), with v′ = v_m / U and r′ = r L / U.
 */
export interface MMGHullCoefficients {
  R0: number;
  Xvv: number;
  Xvr: number;
  Xrr: number;
  Xvvvv: number;
  Yv: number;
  Yr: number;
  Yvvv: number;
  Yvvr: number;
  Yvrr: number;
  Yrrr: number;
  Nv: number;
  Nr: number;
  Nvvv: number;
  Nvvr: number;
  Nvrr: number;
  Nrrr: number;
}

export interface MMGPropeller {
  diameter: number;
  /** KT(J) = k0 + k1 J + k2 J². */
  k0: number;
  k1: number;
  k2: number;
  /** Thrust deduction factor. */
  tP: number;
  /** Effective wake fraction in straight motion. */
  wP0: number;
  C1: number;
  C2Pos: number;
  C2Neg: number;
  /** Propeller position from midship, by L (negative = aft). */
  xP: number;
  /** |KT| astern relative to ahead (simple four-quadrant stand-in). */
  asternFactor: number;
  /** Depth of the shaft below the CG (m, +down) for the pitch coupling of thrust. */
  zBelowCG: number;
}

export interface MMGRudder {
  area: number;
  /** Rudder span (m). */
  span: number;
  tR: number;
  aH: number;
  /** Position of the rudder-induced hull force from midship, by L. */
  xH: number;
  epsilon: number;
  kappa: number;
  gammaRNeg: number;
  gammaRPos: number;
  /** Effective longitudinal coordinate of the rudder in β_R, by L. */
  lR: number;
  /** Rudder lift gradient coefficient fα. */
  fAlpha: number;
  /** Rudder position from midship, by L (−0.5). */
  xR: number;
  /** Depth of the rudder centre below the CG (m). */
  zBelowCG: number;
}

export interface MMGParams {
  length: number;
  draft: number;
  /** CG forward of midship (m). */
  xG: number;
  hull: MMGHullCoefficients;
  propeller: MMGPropeller;
  rudder: MMGRudder;
  /** Added masses (kg) for the Coriolis terms of Yasukawa & Yoshimura Eq. 4. */
  mx: number;
  my: number;
  /** Depth of the hull lateral-force centre below the CG (m) for heel coupling. */
  zHullBelowCG: number;
  /**
   * Frictional correction to R′0 for a different scale than the one it was measured at:
   * R′0(Re) = R′0 − (1 + k)(Cf(Re_ref) − Cf(Re) − ΔCf) S′. Omit to use R′0 as given.
   */
  resistanceScaling?: { referenceReynolds: number; formFactor: number; wettedSurfaceRatio: number; roughnessAllowance: number };
  /** Speed floor for the non-dimensionalisation, as a Froude number (default 0.02). */
  minFroude?: number;
}

export const mmgThrustCoefficient = (p: MMGPropeller, J: number): number => p.k0 + p.k1 * J + p.k2 * J * J;

/** R′0 at a given Reynolds number, applying the optional frictional scaling. */
export function mmgResistanceCoefficient(params: MMGParams, reynolds: number): number {
  const s = params.resistanceScaling;
  if (!s) return params.hull.R0;
  const dCf = schoenherrCf(s.referenceReynolds) - schoenherrCf(reynolds) - s.roughnessAllowance;
  return params.hull.R0 - (1 + s.formFactor) * dCf * s.wettedSurfaceRatio;
}

/** Propeller revolutions (rps) that hold straight-line speed U in calm water. */
export function mmgSelfPropulsionRps(params: MMGParams, U: number, rho: number, nu: number): number {
  const L = params.length;
  const d = params.draft;
  const P = params.propeller;
  const R = mmgResistanceCoefficient(params, (U * L) / nu) * 0.5 * rho * L * d * U * U;
  const target = R / (1 - P.tP);
  const uP = U * (1 - P.wP0);
  const D = P.diameter;
  const thrust = (n: number): number => rho * n * n * D ** 4 * mmgThrustCoefficient(P, uP / (n * D));
  let lo = uP / (D * 1.2);
  let hi = lo * 2;
  while (thrust(hi) < target && hi < 1e5) hi *= 2;
  for (let i = 0; i < 100; i++) {
    const mid = 0.5 * (lo + hi);
    if (thrust(mid) > target) hi = mid;
    else lo = mid;
  }
  return 0.5 * (lo + hi);
}

export function mmgManeuvering(params: MMGParams): ForceModel {
  const { length: L, draft: d, hull: c, propeller: pp, rudder: rr, xG } = params;
  const minFroude = params.minFroude ?? 0.02;
  return {
    name: 'mmg',
    apply(ctx, out) {
      const { rho, nu, g } = ctx.env;
      const u = ctx.relVelBody.x;
      const vG = ctx.relVelBody.y;
      const r = ctx.omegaBody.z;
      const vm = vG - xG * r; // lateral velocity at midship
      const U = Math.hypot(u, vm);
      const Un = Math.max(U, minFroude * Math.sqrt(g * L));
      // drift angle from the floored speed so β → 0 smoothly as the ship stops
      const beta = Math.asin(Math.max(-1, Math.min(1, -vm / Un)));
      const vp = vm / Un;
      const rp = (r * L) / Un;
      const qF = 0.5 * rho * L * d * Un * Un;
      // ---- hull ----
      const R0 = mmgResistanceCoefficient(params, (Math.max(Math.abs(u), 0.01) * L) / nu);
      const Xres = -R0 * 0.5 * rho * L * d * (u * u + vm * vm) * Math.sign(u);
      const XH = Xres + qF * (c.Xvv * vp * vp + c.Xvr * vp * rp + c.Xrr * rp * rp + c.Xvvvv * vp ** 4);
      const YH = qF * (c.Yv * vp + c.Yr * rp + c.Yvvv * vp ** 3 + c.Yvvr * vp * vp * rp + c.Yvrr * vp * rp * rp + c.Yrrr * rp ** 3);
      const NH = qF * L * (c.Nv * vp + c.Nr * rp + c.Nvvv * vp ** 3 + c.Nvvr * vp * vp * rp + c.Nvrr * vp * rp * rp + c.Nrrr * rp ** 3);
      // ---- propeller ----
      const n = ctx.actuators.rps;
      const D = pp.diameter;
      let oneMinusWP = 1 - pp.wP0;
      let KT = 0;
      let JP = 0;
      let T = 0;
      if (Math.abs(n) > 1e-6) {
        const betaP = beta - pp.xP * rp;
        const C2 = betaP > 0 ? pp.C2Pos : pp.C2Neg;
        oneMinusWP = (1 - pp.wP0) * (1 + (1 - Math.exp(-pp.C1 * Math.abs(betaP))) * (C2 - 1));
        if (n > 0) {
          JP = Math.max(-0.5, Math.min(1.2, (u * oneMinusWP) / (n * D)));
          KT = mmgThrustCoefficient(pp, JP);
          T = rho * n * n * D ** 4 * KT;
        } else {
          JP = Math.max(0, Math.min(1.2, -u / (n * D)));
          KT = Math.max(0, mmgThrustCoefficient(pp, JP)) * pp.asternFactor;
          T = -rho * n * n * D ** 4 * KT;
        }
      }
      const XP = (1 - pp.tP) * T;
      // ---- rudder ----
      const delta = ctx.actuators.rudder;
      const betaR = beta - rr.lR * rp;
      const gammaR = betaR < 0 ? rr.gammaRNeg : rr.gammaRPos;
      const vR = Un * gammaR * betaR;
      const uPw = u * oneMinusWP;
      let uR: number;
      if (n > 1e-6 && JP > 1e-6) {
        const eta = D / rr.span;
        const inner = 1 + rr.kappa * (Math.sqrt(1 + (8 * Math.max(KT, 0)) / (Math.PI * JP * JP)) - 1);
        uR = rr.epsilon * uPw * Math.sqrt(eta * inner * inner + (1 - eta));
      } else if (n > 1e-6) {
        // bollard / very low advance: slipstream velocity from momentum theory
        const eta = D / rr.span;
        const uSlip = Math.sqrt((8 * Math.max(KT, 0) * n * n * D * D) / Math.PI);
        const uRP = uPw + rr.kappa * (uSlip - uPw);
        uR = rr.epsilon * Math.sqrt(eta * uRP * uRP + (1 - eta) * uPw * uPw);
      } else {
        uR = rr.epsilon * uPw;
      }
      const UR2 = uR * uR + vR * vR;
      const alphaR = delta - Math.atan2(vR, Math.abs(uR) + 1e-9);
      const FN = 0.5 * rho * rr.area * UR2 * rr.fAlpha * Math.sin(alphaR) * Math.sign(uR || 1);
      const XR = -(1 - rr.tR) * FN * Math.sin(delta);
      const YR = -(1 + rr.aH) * FN * Math.cos(delta);
      const NR = -(rr.xR * L + rr.aH * rr.xH * L) * FN * Math.cos(delta);
      // ---- added-mass Coriolis terms (Y&Y Eq. 4) and transfer to the CG ----
      const XA = params.my * vm * r;
      const YA = -params.mx * u * r;
      const X = XH + XP + XR + XA;
      const Y = YH + YR;
      const Nm = NH + NR;
      out.force.x += X;
      out.force.y += Y + YA;
      // moment about the CG: N_G = N_m − x_G (Y + Y_A), consistent with Y&Y's midship equations
      out.moment.z += Nm - xG * (Y + YA);
      // heel from the hull force at depth and the rudder force at its depth; thrust pitch coupling
      out.moment.x += -params.zHullBelowCG * YH - rr.zBelowCG * YR;
      out.moment.y += pp.zBelowCG * XP;
      const dg = ctx.diag;
      dg['mmg.beta'] = beta;
      dg['mmg.rp'] = rp;
      dg['mmg.thrust'] = T;
      dg['mmg.KT'] = KT;
      dg['mmg.J'] = JP;
      dg['mmg.rudderNormalForce'] = FN;
      dg['mmg.XH'] = XH;
      dg['mmg.YH'] = YH;
      dg['mmg.NH'] = NH;
    },
  };
}
