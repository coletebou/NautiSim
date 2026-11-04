import { ALL_DOF, dof6, type Vessel } from '../core/types';
import { hydrostaticForce, panelCrossflow, viscousDamping, windage } from '../forces/basic';
import { lateralHull, outboardDrive } from '../forces/propulsion';
import { preplaningResistance, savitskyPlaning } from '../forces/savitsky';
import { translateMesh } from '../geometry/mesh';
import { createPlaningHull } from '../geometry/planingHull';
import { DEG_IN_RAD, HP_IN_W, RHO_SEAWATER } from '../units';
import { dampingFromRatios, estimateRestoring } from './common';

/** Data describing a planing boat. Everything the builder needs, nothing it computes. */
export interface PlaningBoatDefinition {
  id: string;
  name: string;
  description: string;
  length: number;
  chineBeam: number;
  sheerBeam: number;
  deadriseDeg: number;
  depth: number;
  /** Loaded mass including crew and fuel (kg). */
  mass: number;
  /** CG forward of the transom (m) and above the keel (m). */
  lcg: number;
  vcg: number;
  /** Radii of gyration over beam (roll) and length (pitch, yaw). */
  kxxOverB: number;
  kyyOverL: number;
  kzzOverL: number;
  engine: {
    powerHp: number;
    ratedRpm: number;
    gearRatio: number;
    propDiameter: number;
    pitchRatio: number;
    /** Propeller below the transom keel (m) and aft of the transom (m). */
    propDepthBelowKeel: number;
    propAftOfTransom: number;
    thrustAngleDeg: number;
    legArea: number;
    maxSteerDeg: number;
  };
  windage: { frontalArea: number; lateralArea: number; centroidAboveKeel: number };
  /** Bridge eye position: forward of the transom (m) and above the keel (m). */
  eye: { x: number; height: number };
  /** Damping ratios for heave, roll and pitch. */
  zeta: { heave: number; roll: number; pitch: number };
}

export const RIB_65: PlaningBoatDefinition = {
  id: 'rib',
  name: 'RIB 6.5 m',
  description: '6.5 m rigid-inflatable with a 22° deep-V and a 150 hp outboard. Quick, wet and lively in a chop.',
  length: 6.5,
  chineBeam: 2.0,
  sheerBeam: 2.5,
  deadriseDeg: 22,
  depth: 1.0,
  mass: 1600,
  lcg: 2.4,
  vcg: 0.55,
  kxxOverB: 0.36,
  kyyOverL: 0.26,
  kzzOverL: 0.27,
  engine: {
    powerHp: 150,
    ratedRpm: 5500,
    gearRatio: 2.0,
    propDiameter: 0.36,
    pitchRatio: 1.4,
    propDepthBelowKeel: 0.25,
    propAftOfTransom: 0.45,
    thrustAngleDeg: 0,
    legArea: 0.05,
    maxSteerDeg: 25,
  },
  windage: { frontalArea: 3.2, lateralArea: 7.5, centroidAboveKeel: 1.3 },
  eye: { x: 3.0, height: 1.9 },
  zeta: { heave: 0.35, roll: 0.5, pitch: 0.35 },
};

export const RUNABOUT_8: PlaningBoatDefinition = {
  id: 'runabout',
  name: 'Sport runabout 8 m',
  description: '8 m bowrider with a 20° deep-V and a 300 hp sterndrive. Fast, dry and stable.',
  length: 8.0,
  chineBeam: 2.35,
  sheerBeam: 2.6,
  deadriseDeg: 20,
  depth: 1.25,
  mass: 3300,
  lcg: 2.9,
  vcg: 0.7,
  kxxOverB: 0.35,
  kyyOverL: 0.25,
  kzzOverL: 0.26,
  engine: {
    powerHp: 300,
    ratedRpm: 5000,
    gearRatio: 1.8,
    propDiameter: 0.4,
    pitchRatio: 1.3,
    propDepthBelowKeel: 0.3,
    propAftOfTransom: 0.6,
    thrustAngleDeg: 0,
    legArea: 0.07,
    maxSteerDeg: 25,
  },
  windage: { frontalArea: 4.5, lateralArea: 11, centroidAboveKeel: 1.5 },
  eye: { x: 3.6, height: 2.1 },
  zeta: { heave: 0.35, roll: 0.5, pitch: 0.35 },
};

export interface PlaningVessel extends Vessel {
  readonly definition: PlaningBoatDefinition;
  readonly transomKeel: { x: number; y: number; z: number };
}

export function createPlaningBoat(def: PlaningBoatDefinition, opts: { rho?: number; g?: number; resolution?: 'physics' | 'display' } = {}): PlaningVessel {
  const rho = opts.rho ?? RHO_SEAWATER;
  const g = opts.g ?? 9.81;
  const display = opts.resolution === 'display';
  const raw = createPlaningHull({
    length: def.length,
    chineBeam: def.chineBeam,
    sheerBeam: def.sheerBeam,
    deadrise: def.deadriseDeg * DEG_IN_RAD,
    depth: def.depth,
    stations: display ? 49 : 25,
    pointsBottom: display ? 8 : 4,
    pointsSide: display ? 4 : 2,
  });
  // generator: transom keel at the origin, z down; the CG is lcg forward and vcg above the keel
  const cg = { x: def.lcg, y: 0, z: -def.vcg };
  const mesh = translateMesh(raw, { x: -cg.x, y: 0, z: -cg.z });
  const transomKeel = { x: -def.lcg, y: 0, z: def.vcg };
  const prismaticLength = 0.55 * def.length;
  const m = def.mass;
  const Ixx = m * (def.kxxOverB * def.sheerBeam) ** 2;
  const Iyy = m * (def.kyyOverL * def.length) ** 2;
  const Izz = m * (def.kzzOverL * def.length) ** 2;
  const added = dof6(0.05 * m, 0.4 * m, 0.5 * m, 0.15 * Ixx, 0.5 * Iyy, 0.3 * Izz);
  const restoring = estimateRestoring(mesh, m, rho, g);
  const damp = dampingFromRatios(restoring, m, { x: Ixx, y: Iyy }, added, def.zeta);
  const E = def.engine;
  const rpsMax = E.ratedRpm / 60 / E.gearRatio;
  const propPos = { x: transomKeel.x - E.propAftOfTransom, y: 0, z: transomKeel.z + E.propDepthBelowKeel };
  return {
    id: def.id,
    name: def.name,
    mass: m,
    inertia: { x: Ixx, y: Iyy, z: Izz },
    addedMass: added,
    mesh,
    forceModels: [
      hydrostaticForce(),
      viscousDamping({
        linear: { heave: damp.heave, roll: damp.roll, pitch: damp.pitch, yaw: 0.05 * Izz },
        quadratic: { roll: 0.3 * Ixx, yaw: 0.3 * Izz },
      }),
      savitskyPlaning({
        beam: def.chineBeam,
        deadrise: def.deadriseDeg * DEG_IN_RAD,
        transomKeel,
        maxWettedLength: 0.85 * def.length,
        forefoot: { prismaticLength, length: def.length, rise: 0.45 * def.depth, deadriseAtStem: Math.min(1.1, def.deadriseDeg * DEG_IN_RAD * 2.2) },
      }),
      preplaningResistance({ weight: m * g, volume: m / rho, pitchStiffness: restoring.kPitch, humpTrim: 1.5 * DEG_IN_RAD }),
      lateralHull({ transomKeel, length: 0.85 * def.length, crossflowCd: 0 }),
      panelCrossflow(),
      outboardDrive({
        diameter: E.propDiameter,
        pitchRatio: E.pitchRatio,
        position: propPos,
        ratedPower: E.powerHp * HP_IN_W,
        thrustAngle: E.thrustAngleDeg * DEG_IN_RAD,
        legArea: E.legArea,
      }),
      windage({
        frontalArea: def.windage.frontalArea,
        lateralArea: def.windage.lateralArea,
        centroid: { x: 0, y: 0, z: -(def.windage.centroidAboveKeel - def.vcg) },
      }),
    ],
    actuators: {
      rudderMax: E.maxSteerDeg * DEG_IN_RAD,
      rudderRate: 40 * DEG_IN_RAD,
      rpsMax,
      asternFraction: 0.5,
      rpsRate: rpsMax / 1.5,
    },
    dofMask: ALL_DOF,
    crossflowCd: 1.0,
    fixedStep: 1 / 60,
    dimensions: { length: def.length, beam: def.sheerBeam, draft: restoring.rest.z + def.vcg, depth: def.depth },
    bridgeEye: { x: def.eye.x - def.lcg, y: 0, z: -(def.eye.height - def.vcg) },
    gearRatio: E.gearRatio,
    rest: restoring.rest,
    meta: {
      description: def.description,
      model: 'Savitsky (1964) planing + mesh hydrostatics',
      powerHp: E.powerHp,
      gm: restoring.gm,
      prismaticLength,
    },
    definition: def,
    transomKeel,
  };
}
