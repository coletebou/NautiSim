import type { EnvironmentSampler } from '../core/environment';
import { createIntegrator, type Integrator, type IntegratorKind } from '../core/integrators';
import { type BodyProperties, packState, STATE_SIZE, unpackState } from '../core/rigidbody';
import type { ActuatorState, Controls, ForceContext, Vessel, VesselState, Wrench } from '../core/types';
import { computeHydrostatics, createHydroScratch, type HydroResult, type HydroScratch, newHydroResult } from '../hydrostatics/hydrostatics';
import { type Euler, quatFromEuler, quatToEuler, rotate, rotateInverse } from '../math/quat';
import { approach, clamp, wrap2Pi } from '../math/scalar';
import type { Vec3 } from '../math/vec3';

/** What the HUD and renderer read after each step. Plain data, safe to copy across threads. */
export interface VesselSnapshot {
  t: number;
  step: number;
  position: Vec3;
  orientation: { w: number; x: number; y: number; z: number };
  euler: Euler;
  /** Heading 0…2π, clockwise from north. */
  heading: number;
  velBody: Vec3;
  omegaBody: Vec3;
  /** Speed over ground (m/s). */
  speedOverGround: number;
  /** Speed through the water (m/s). */
  speedThroughWater: number;
  courseOverGround: number;
  actuators: ActuatorState;
  /** Engine revolutions per minute. */
  engineRpm: number;
  /** Vertical acceleration at the CG in the body frame (m/s², without gravity), for haptics. */
  heaveAcceleration: number;
  diag: Record<string, number>;
}

export interface SimulationOptions {
  integrator?: IntegratorKind;
  /** Override the vessel's recommended step. */
  fixedStep?: number;
  initial?: Partial<{ position: Vec3; heading: number; speed: number }>;
}

interface MutableContext {
  t: number;
  vessel: Vessel;
  env: EnvironmentSampler;
  actuators: ActuatorState;
  position: Vec3;
  orientation: { w: number; x: number; y: number; z: number };
  euler: Euler;
  velBody: Vec3;
  omegaBody: Vec3;
  waterVelBody: Vec3;
  relVelBody: Vec3;
  windRelBody: Vec3;
  hydro: HydroResult;
  diag: Record<string, number>;
}

/**
 * Deterministic fixed-step simulation of one vessel in an environment. `step()` advances by
 * exactly `dt`; time comes from the step counter, never from a clock.
 */
export class Simulation {
  readonly vessel: Vessel;
  readonly env: EnvironmentSampler;
  readonly dt: number;
  private readonly integrator: Integrator;
  private readonly body: BodyProperties;
  private readonly y = new Float64Array(STATE_SIZE);
  private readonly hydroScratch: HydroScratch;
  private readonly ctx: MutableContext;
  private readonly tmpVec: Vec3 = { x: 0, y: 0, z: 0 };
  private stepCount = 0;
  private lastHeaveAccel = 0;
  private prevW = 0;
  controls: Controls = { throttle: 0, helm: 0, thruster: 0 };
  readonly actuators: ActuatorState = { rudder: 0, rps: 0, thruster: 0, throttle: 0 };

  constructor(vessel: Vessel, env: EnvironmentSampler, opts: SimulationOptions = {}) {
    this.vessel = vessel;
    this.env = env;
    this.dt = opts.fixedStep ?? vessel.fixedStep;
    this.integrator = createIntegrator(opts.integrator ?? 'rk4');
    this.body = { mass: vessel.mass, inertia: vessel.inertia, addedMass: vessel.addedMass, g: env.g, mask: vessel.dofMask };
    this.hydroScratch = createHydroScratch(vessel.mesh);
    this.ctx = {
      t: 0,
      vessel,
      env,
      actuators: this.actuators,
      position: { x: 0, y: 0, z: 0 },
      orientation: { w: 1, x: 0, y: 0, z: 0 },
      euler: { roll: 0, pitch: 0, yaw: 0 },
      velBody: { x: 0, y: 0, z: 0 },
      omegaBody: { x: 0, y: 0, z: 0 },
      waterVelBody: { x: 0, y: 0, z: 0 },
      relVelBody: { x: 0, y: 0, z: 0 },
      windRelBody: { x: 0, y: 0, z: 0 },
      hydro: newHydroResult(),
      diag: {},
    };
    this.reset(opts.initial);
  }

  get time(): number {
    return this.stepCount * this.dt;
  }

  get steps(): number {
    return this.stepCount;
  }

  /** Place the vessel at rest (or at a given speed) in its calm-water equilibrium attitude. */
  reset(initial: SimulationOptions['initial'] = {}): void {
    const r = this.vessel.rest;
    const heading = initial.heading ?? 0;
    const pos = initial.position ?? { x: 0, y: 0, z: 0 };
    packState(
      this.y,
      { x: pos.x, y: pos.y, z: r.z + pos.z },
      quatFromEuler(r.roll, r.pitch, heading),
      { x: initial.speed ?? 0, y: 0, z: 0 },
      { x: 0, y: 0, z: 0 },
    );
    this.stepCount = 0;
    this.actuators.rudder = 0;
    this.actuators.thruster = 0;
    this.actuators.throttle = 0;
    this.actuators.rps = 0;
    this.controls = { throttle: 0, helm: 0, thruster: 0 };
    this.prevW = 0;
    this.lastHeaveAccel = 0;
  }

  /** Overwrite the kinematic state (for tests, replays and network sync). */
  setState(s: Readonly<VesselState>): void {
    packState(this.y, s.position, s.orientation, s.velBody, s.omegaBody);
  }

  getState(): VesselState {
    return unpackState(this.y);
  }

  /** Set the propeller rps directly (validation runs hold a constant rps). */
  setRps(rps: number): void {
    this.actuators.rps = rps;
  }

  private updateActuators(h: number): void {
    const spec = this.vessel.actuators;
    const c = this.controls;
    const targetRudder = clamp(c.helm, -1, 1) * spec.rudderMax;
    this.actuators.rudder = approach(this.actuators.rudder, targetRudder, spec.rudderRate * h);
    const th = clamp(c.throttle, -1, 1);
    this.actuators.throttle = th;
    const targetRps = th >= 0 ? th * spec.rpsMax : th * spec.rpsMax * spec.asternFraction;
    this.actuators.rps = approach(this.actuators.rps, targetRps, spec.rpsRate * h);
    this.actuators.thruster = approach(this.actuators.thruster, clamp(c.thruster, -1, 1), 0.5 * h);
  }

  /** Hold actuators exactly at a set of values (bypassing rate limits) for scripted manoeuvres. */
  overrideActuators(a: Partial<ActuatorState>): void {
    Object.assign(this.actuators, a);
  }

  private readonly wrenchFn = (y: Float64Array, t: number, out: Wrench): void => {
    const ctx = this.ctx;
    ctx.t = t;
    ctx.position.x = y[0] as number;
    ctx.position.y = y[1] as number;
    ctx.position.z = y[2] as number;
    const q = ctx.orientation;
    q.w = y[3] as number;
    q.x = y[4] as number;
    q.y = y[5] as number;
    q.z = y[6] as number;
    const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
    const e = quatToEuler(q);
    ctx.euler.roll = e.roll;
    ctx.euler.pitch = e.pitch;
    ctx.euler.yaw = e.yaw;
    ctx.velBody.x = y[7] as number;
    ctx.velBody.y = y[8] as number;
    ctx.velBody.z = y[9] as number;
    ctx.omegaBody.x = y[10] as number;
    ctx.omegaBody.y = y[11] as number;
    ctx.omegaBody.z = y[12] as number;
    const env = this.env;
    const p = ctx.position;
    const tmp = this.tmpVec;
    env.waterVelocity(p.x, p.y, Math.max(p.z, 0), t, tmp);
    rotateInverse(q, tmp, ctx.waterVelBody);
    ctx.relVelBody.x = ctx.velBody.x - ctx.waterVelBody.x;
    ctx.relVelBody.y = ctx.velBody.y - ctx.waterVelBody.y;
    ctx.relVelBody.z = ctx.velBody.z - ctx.waterVelBody.z;
    env.wind(p.x, p.y, t, tmp);
    const windBody = rotateInverse(q, tmp, this.scratchWind);
    ctx.windRelBody.x = windBody.x - ctx.velBody.x;
    ctx.windRelBody.y = windBody.y - ctx.velBody.y;
    ctx.windRelBody.z = 0;
    const cd = this.vessel.crossflowCd ?? 0;
    if (cd > 0) {
      rotate(q, ctx.relVelBody, this.kin.velocity);
      rotate(q, ctx.omegaBody, this.kin.omega);
      this.kin.cd = cd;
      computeHydrostatics(this.vessel.mesh, p, q, env, t, this.hydroScratch, ctx.hydro, this.kin);
    } else {
      computeHydrostatics(this.vessel.mesh, p, q, env, t, this.hydroScratch, ctx.hydro);
    }
    const models = this.vessel.forceModels;
    const fctx = ctx as ForceContext;
    if (this.debugForces) {
      for (let i = 0; i < models.length; i++) {
        const m = models[i] as (typeof models)[number];
        const fz0 = out.force.z;
        const my0 = out.moment.y;
        m.apply(fctx, out);
        ctx.diag[`F.${m.name}.z`] = out.force.z - fz0;
        ctx.diag[`F.${m.name}.my`] = out.moment.y - my0;
      }
    } else {
      for (let i = 0; i < models.length; i++) (models[i] as (typeof models)[number]).apply(fctx, out);
    }
  };
  /** Record each force model's body-z force and pitch moment in the diagnostics (slow). */
  debugForces = false;
  private readonly scratchWind: Vec3 = { x: 0, y: 0, z: 0 };
  private readonly kin = { velocity: { x: 0, y: 0, z: 0 }, omega: { x: 0, y: 0, z: 0 }, cd: 0 };

  /** Advance one fixed step. */
  step(): void {
    const h = this.dt;
    this.updateActuators(h);
    const t = this.time;
    this.integrator.step(this.y, t, h, this.body, this.wrenchFn);
    this.stepCount++;
    const w = this.y[9] as number;
    this.lastHeaveAccel = (w - this.prevW) / h;
    this.prevW = w;
  }

  /** Advance n fixed steps. */
  run(n: number): void {
    for (let i = 0; i < n; i++) this.step();
  }

  snapshot(): VesselSnapshot {
    const s = unpackState(this.y);
    const euler = quatToEuler(s.orientation);
    const velWorld = rotate(s.orientation, s.velBody);
    const water = this.env.waterVelocity(s.position.x, s.position.y, Math.max(s.position.z, 0), this.time, { x: 0, y: 0, z: 0 });
    const relWorld = { x: velWorld.x - water.x, y: velWorld.y - water.y };
    const spec = this.vessel;
    return {
      t: this.time,
      step: this.stepCount,
      position: s.position,
      orientation: s.orientation,
      euler,
      heading: wrap2Pi(euler.yaw),
      velBody: s.velBody,
      omegaBody: s.omegaBody,
      speedOverGround: Math.hypot(velWorld.x, velWorld.y),
      speedThroughWater: Math.hypot(relWorld.x, relWorld.y),
      courseOverGround: wrap2Pi(Math.atan2(velWorld.y, velWorld.x)),
      actuators: { ...this.actuators },
      engineRpm: Math.abs(this.ctx.diag['engine.rps'] ?? this.actuators.rps) * 60 * spec.gearRatio,
      heaveAcceleration: this.lastHeaveAccel,
      diag: { ...this.ctx.diag },
    };
  }
}
