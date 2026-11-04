import { ALL_DOF, type DofMask, dof6, PLANAR_DOF, type Vessel } from '../core/types';
import { bowThruster, hydrostaticForce, viscousDamping, windage } from '../forces/basic';
import { type MMGParams, mmgManeuvering, mmgSelfPropulsionRps } from '../forces/mmg';
import { translateMesh } from '../geometry/mesh';
import { createShipHull } from '../geometry/shipHull';
import { DEG_IN_RAD, KNOT_IN_MPS, NU_SEAWATER_15C, RHO_SEAWATER } from '../units';
import { dampingFromRatios, estimateRestoring } from './common';

/**
 * KVLCC2 (KRISO Very Large Crude Carrier 2) data. Principal particulars and MMG coefficients from
 * Yasukawa & Yoshimura (2015), Tables 1 and 3; KG and wetted surface from the SIMMAN 2008 KVLCC2
 * particulars. Values marked "estimate" are NautiSim's own (no published source) and only affect
 * seakeeping and the bridge view, not the manoeuvring validation.
 */
export const KVLCC2_DATA = {
  fullScale: {
    Lpp: 320.0,
    B: 58.0,
    d: 20.8,
    depth: 30.0,
    displacementVolume: 312_600,
    xG: 11.2,
    KG: 18.6,
    propellerDiameter: 9.86,
    rudderSpan: 15.8,
    rudderArea: 112.5,
    wettedSurface: 27_194,
  },
  /** Geometric scale of the L7 free-running model used for validation. */
  modelScale: 45.714,
  hull: {
    R0: 0.022,
    Xvv: -0.04,
    Xvr: 0.002,
    Xrr: 0.011,
    Xvvvv: 0.771,
    Yv: -0.315,
    Yr: 0.083,
    Yvvv: -1.607,
    Yvvr: 0.379,
    Yvrr: -0.391,
    Yrrr: 0.008,
    Nv: -0.137,
    Nr: -0.049,
    Nvvv: -0.03,
    Nvvr: -0.294,
    Nvrr: 0.055,
    Nrrr: -0.013,
  },
  addedMass: { mx: 0.022, my: 0.223, Jz: 0.011 },
  propeller: { k0: 0.2931, k1: -0.2753, k2: -0.1385, tP: 0.22, wP0Model: 0.4, wP0Full: 0.35, C1: 2.0, C2Pos: 1.6, C2Neg: 1.1, xP: -0.48 },
  rudder: { tR: 0.387, aH: 0.312, xH: -0.464, epsilon: 1.09, kappa: 0.5, gammaRNeg: 0.395, gammaRPos: 0.64, lR: -0.71, fAlpha: 2.747, xR: -0.5 },
  kzzOverL: 0.25,
  /** Roll radius of gyration / B and pitch / L (estimate, typical tanker values). */
  kxxOverB: 0.4,
  kyyOverL: 0.25,
  /** Approach speed of the benchmark manoeuvres. */
  approachSpeedKnots: 15.5,
  /** Rudder rate, full scale (Froude-scaled to 15.8 °/s on the L7 model). */
  rudderRateDegPerSecFull: 2.34,
  /** Estimates: vertical-plane added mass fractions and damping ratios. */
  vertical: { heaveAddedMass: 0.9, rollAddedInertia: 0.15, pitchAddedInertia: 0.9, zetaHeave: 0.25, zetaRoll: 0.08, zetaPitch: 0.25 },
  /** Estimates: above-water areas, full scale. */
  windage: { frontalArea: 1250, lateralArea: 4600, centroidAboveWL: 13, centroidXFromMidship: -40 },
} as const;

export interface KVLCC2Options {
  /** Geometric scale λ (1 = full scale; 45.714 = the L7 model). */
  scale?: number;
  /** 'planar' locks heave/roll/pitch (pure MMG); '6dof' integrates all six. */
  dof?: 'planar' | '6dof';
  rho?: number;
  nu?: number;
  g?: number;
  /** Mesh resolution. */
  resolution?: 'physics' | 'display';
}

export interface KVLCC2Vessel extends Vessel {
  readonly mmg: MMGParams;
  /** Approach speed (m/s) and the rps that holds it. */
  readonly approachSpeed: number;
  readonly approachRps: number;
}

export function createKVLCC2(opts: KVLCC2Options = {}): KVLCC2Vessel {
  const D = KVLCC2_DATA;
  const lam = opts.scale ?? 1;
  const rho = opts.rho ?? RHO_SEAWATER;
  const nu = opts.nu ?? NU_SEAWATER_15C;
  const g = opts.g ?? 9.81;
  const F = D.fullScale;
  const L = F.Lpp / lam;
  const B = F.B / lam;
  const d = F.d / lam;
  const depth = F.depth / lam;
  const volume = F.displacementVolume / lam ** 3;
  const xG = F.xG / lam;
  const KG = F.KG / lam;
  const mass = rho * volume;
  const display = opts.resolution === 'display';
  const geo = createShipHull({
    length: L,
    beam: B,
    draft: d,
    depth,
    blockCoefficient: volume / (L * B * d),
    lcb: xG,
    stations: display ? 81 : 41,
    pointsBelow: display ? 14 : 8,
    pointsAbove: display ? 3 : 2,
  });
  // generator origin: midship on the waterline (z down) → move it to the CG
  const cg = { x: xG, y: 0, z: d - KG };
  const mesh = translateMesh(geo.mesh, { x: -cg.x, y: -cg.y, z: -cg.z });
  const Ixx = mass * (D.kxxOverB * B) ** 2;
  const Iyy = mass * (D.kyyOverL * L) ** 2;
  const Izz = mass * (D.kzzOverL * L) ** 2;
  const m2 = 0.5 * rho * L * L * d;
  const V = D.vertical;
  const added = dof6(D.addedMass.mx * m2, D.addedMass.my * m2, V.heaveAddedMass * mass, V.rollAddedInertia * Ixx, V.pitchAddedInertia * Iyy, D.addedMass.Jz * 0.5 * rho * L ** 4 * d);
  const restoring = estimateRestoring(mesh, mass, rho, g);
  const damp = dampingFromRatios(restoring, mass, { x: Ixx, y: Iyy }, added, { heave: V.zetaHeave, roll: V.zetaRoll, pitch: V.zetaPitch });
  const P = D.propeller;
  const R = D.rudder;
  const DP = F.propellerDiameter / lam;
  const HR = F.rudderSpan / lam;
  const refL = F.Lpp / D.modelScale;
  const refU = (D.approachSpeedKnots * KNOT_IN_MPS) / Math.sqrt(D.modelScale);
  const isModel = Math.abs(lam - D.modelScale) / D.modelScale < 0.05;
  const mmg: MMGParams = {
    length: L,
    draft: d,
    xG,
    hull: { ...D.hull },
    propeller: {
      diameter: DP,
      k0: P.k0,
      k1: P.k1,
      k2: P.k2,
      tP: P.tP,
      wP0: lam > 1.5 ? P.wP0Model : P.wP0Full,
      C1: P.C1,
      C2Pos: P.C2Pos,
      C2Neg: P.C2Neg,
      xP: P.xP,
      asternFactor: 0.6,
      zBelowCG: d - DP / 2 - 0.05 * d - (d - KG),
    },
    rudder: {
      area: F.rudderArea / lam ** 2,
      span: HR,
      tR: R.tR,
      aH: R.aH,
      xH: R.xH,
      epsilon: R.epsilon,
      kappa: R.kappa,
      gammaRNeg: R.gammaRNeg,
      gammaRPos: R.gammaRPos,
      lR: R.lR,
      fAlpha: R.fAlpha,
      xR: R.xR,
      zBelowCG: d - HR / 2 - (d - KG),
    },
    mx: added.surge,
    my: added.sway,
    zHullBelowCG: KG - d / 2,
    // R′0 = 0.022 was measured on the L7 model; other scales get the Schoenherr frictional correction
    ...(!isModel && {
      resistanceScaling: {
        referenceReynolds: (refU * refL) / nu,
        formFactor: 0.2,
        wettedSurfaceRatio: F.wettedSurface / (F.Lpp * F.d),
        roughnessAllowance: lam < 2 ? 0.0002 : 0,
      },
    }),
  };
  const approachSpeed = (D.approachSpeedKnots * KNOT_IN_MPS) / Math.sqrt(lam);
  const approachRps = mmgSelfPropulsionRps(mmg, approachSpeed, rho, nu);
  const W = D.windage;
  const mask: DofMask = opts.dof === 'planar' ? PLANAR_DOF : ALL_DOF;
  const rpsMax = 1.15 * approachRps;
  return {
    id: lam === 1 ? 'kvlcc2' : `kvlcc2-1:${lam}`,
    name: lam === 1 ? 'KVLCC2 tanker' : `KVLCC2 1:${lam} model`,
    mass,
    inertia: { x: Ixx, y: Iyy, z: Izz },
    addedMass: added,
    mesh,
    forceModels: [
      hydrostaticForce(),
      viscousDamping({ linear: { heave: damp.heave, roll: damp.roll, pitch: damp.pitch } }),
      mmgManeuvering(mmg),
      windage({
        frontalArea: W.frontalArea / lam ** 2,
        lateralArea: W.lateralArea / lam ** 2,
        centroid: { x: W.centroidXFromMidship / lam - xG, y: 0, z: -(W.centroidAboveWL / lam) + (d - KG) },
      }),
      bowThruster({ maxForce: 4.0e5 / lam ** 3, position: { x: 0.45 * L - xG, y: 0, z: d / 2 - (d - KG) }, cutoffSpeed: 2.5 / Math.sqrt(lam) }),
    ],
    actuators: {
      rudderMax: 35 * DEG_IN_RAD,
      rudderRate: D.rudderRateDegPerSecFull * DEG_IN_RAD * Math.sqrt(lam),
      rpsMax,
      asternFraction: 0.7,
      rpsRate: rpsMax / (90 / Math.sqrt(lam)),
    },
    dofMask: mask,
    fixedStep: (1 / 30) / Math.sqrt(lam),
    dimensions: { length: L, beam: B, draft: d, depth },
    bridgeEye: { x: -0.4 * L - xG, y: 0, z: -(depth - d) - 25 / lam + (d - KG) },
    gearRatio: 1,
    rest: restoring.rest,
    meta: {
      description: '320 m very large crude carrier. Manoeuvring by the MMG standard method with the published KVLCC2 coefficients.',
      model: 'MMG (Yasukawa & Yoshimura 2015)',
      cruiseKnots: D.approachSpeedKnots,
      gm: restoring.gm,
    },
    mmg,
    approachSpeed,
    approachRps,
  };
}
