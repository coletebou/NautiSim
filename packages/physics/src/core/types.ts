import type { TriangleMesh } from '../geometry/mesh';
import type { HydroResult } from '../hydrostatics/hydrostatics';
import type { Euler, Quat } from '../math/quat';
import type { Vec3 } from '../math/vec3';
import type { EnvironmentSampler } from './environment';

/** Six components keyed by SNAME degree of freedom. */
export interface Dof6 {
  surge: number;
  sway: number;
  heave: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export const dof6 = (surge = 0, sway = 0, heave = 0, roll = 0, pitch = 0, yaw = 0): Dof6 => ({
  surge,
  sway,
  heave,
  roll,
  pitch,
  yaw,
});

/** Which degrees of freedom are integrated. Locked DOFs keep zero velocity. */
export type DofMask = Readonly<Record<keyof Dof6, boolean>>;
export const ALL_DOF: DofMask = { surge: true, sway: true, heave: true, roll: true, pitch: true, yaw: true };
export const PLANAR_DOF: DofMask = { surge: true, sway: true, heave: false, roll: false, pitch: false, yaw: true };

/** Normalised operator commands (from the helm, an autopilot or a test script). */
export interface Controls {
  /** −1 (full astern) … 1 (full ahead). */
  throttle: number;
  /** −1 (full port) … 1 (full starboard). Positive turns the vessel to starboard. */
  helm: number;
  /** −1 … 1 bow thruster, positive pushes the bow to starboard. */
  thruster: number;
}

export const neutralControls = (): Controls => ({ throttle: 0, helm: 0, thruster: 0 });

/** Actuator state after rate limits. */
export interface ActuatorState {
  /** Rudder or outboard steering angle (rad), positive turns to starboard. */
  rudder: number;
  /** Propeller revolutions per second, signed (negative = astern). */
  rps: number;
  /** Bow thruster command −1…1. */
  thruster: number;
  /** Throttle as last commanded, −1…1 (used by power-limited engines). */
  throttle: number;
}

export interface ActuatorSpec {
  /** Maximum rudder / steering angle (rad). */
  rudderMax: number;
  /** Rudder slew rate (rad/s). */
  rudderRate: number;
  /** Maximum ahead propeller rps. */
  rpsMax: number;
  /** Astern rps as a fraction of rpsMax. */
  asternFraction: number;
  /** rps slew rate (rps/s). */
  rpsRate: number;
}

/** Force and moment in the body frame, moment about the centre of gravity. */
export interface Wrench {
  force: Vec3;
  moment: Vec3;
}

export const newWrench = (): Wrench => ({ force: { x: 0, y: 0, z: 0 }, moment: { x: 0, y: 0, z: 0 } });

/** Everything a force model may read during one evaluation. Treat as read-only. */
export interface ForceContext {
  readonly t: number;
  readonly vessel: Vessel;
  readonly env: EnvironmentSampler;
  readonly actuators: Readonly<ActuatorState>;
  /** World (NED) position of the CG. */
  readonly position: Readonly<Vec3>;
  readonly orientation: Readonly<Quat>;
  readonly euler: Readonly<Euler>;
  /** Body-frame velocity of the CG over ground. */
  readonly velBody: Readonly<Vec3>;
  /** Body-frame angular velocity (p, q, r). */
  readonly omegaBody: Readonly<Vec3>;
  /** Water velocity (current + orbital) at the CG, body frame. */
  readonly waterVelBody: Readonly<Vec3>;
  /** Velocity of the CG relative to the water, body frame. */
  readonly relVelBody: Readonly<Vec3>;
  /** Apparent wind (wind minus vessel velocity), body frame. */
  readonly windRelBody: Readonly<Vec3>;
  /** Hydrostatic integration for this evaluation (shared by all models). */
  readonly hydro: Readonly<HydroResult>;
  /** Scratch for diagnostics; copied into the simulation snapshot after each step. */
  readonly diag: Record<string, number>;
}

export interface ForceModel {
  readonly name: string;
  /** Add this model's force and moment (body frame, about the CG) into `out`. */
  apply(ctx: ForceContext, out: Wrench): void;
}

/** A runtime vessel: mass properties, geometry and force models. Built from a HullDefinition. */
export interface Vessel {
  readonly id: string;
  readonly name: string;
  readonly mass: number;
  /** Principal moments of inertia about the CG (kg·m²). */
  readonly inertia: Readonly<Vec3>;
  /** Diagonal added mass (kg, kg·m²). */
  readonly addedMass: Readonly<Dof6>;
  /** Closed hull mesh in the body frame, origin at the CG (used for hydrostatics). */
  readonly mesh: TriangleMesh;
  readonly forceModels: readonly ForceModel[];
  readonly actuators: Readonly<ActuatorSpec>;
  readonly dofMask: DofMask;
  /** Recommended fixed physics step (s). */
  readonly fixedStep: number;
  readonly dimensions: Readonly<{ length: number; beam: number; draft: number; depth: number }>;
  /** Body-frame eye position for the bridge camera, relative to the CG (m). */
  readonly bridgeEye: Readonly<Vec3>;
  /** Engine revolutions per propeller revolution (for the RPM gauge). */
  readonly gearRatio: number;
  /** Initial state in calm water (CG depth and pitch at equilibrium). */
  readonly rest: Readonly<{ z: number; pitch: number; roll: number }>;
  /** Free-form metadata for the UI (descriptions, cruise speed…). */
  readonly meta: Readonly<Record<string, string | number>>;
}

/** Kinematic state of the vessel. */
export interface VesselState {
  position: Vec3;
  orientation: Quat;
  velBody: Vec3;
  omegaBody: Vec3;
}
