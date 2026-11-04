import { wrapPi } from '../math/scalar';
import type { Simulation } from '../sim/simulation';

/**
 * Standard IMO / ITTC manoeuvres, scripted on a Simulation. The vessel starts on a straight
 * course at `speed` with the propeller held at `rps`, then the helm is applied at the vessel's
 * rudder rate.
 */
export interface ManeuverStart {
  speed: number;
  rps: number;
}

export interface TrackPoint {
  t: number;
  x: number;
  y: number;
  heading: number;
  rudder: number;
  speed: number;
}

export interface TurningResult {
  /** Advance: distance along the approach course at 90° heading change (m). */
  advance: number;
  /** Transfer: lateral distance at 90° heading change (m). */
  transfer: number;
  /** Tactical diameter: lateral distance at 180° heading change (m). */
  tacticalDiameter: number;
  /** Steady turning diameter from the second half-circle (m). */
  steadyDiameter: number;
  /** Speed at 360° of turn over approach speed. */
  speedLossRatio: number;
  track: TrackPoint[];
}

function startStraight(sim: Simulation, start: ManeuverStart): void {
  sim.reset({ speed: start.speed, heading: 0 });
  const spec = sim.vessel.actuators;
  sim.controls = { throttle: start.rps / spec.rpsMax, helm: 0, thruster: 0 };
  sim.overrideActuators({ rps: start.rps, rudder: 0 });
}

function sample(sim: Simulation, unwrapped: number): TrackPoint {
  const s = sim.snapshot();
  return { t: s.t, x: s.position.x, y: s.position.y, heading: unwrapped, rudder: s.actuators.rudder, speed: s.speedThroughWater };
}

/** Turning circle with rudder angle δ (rad, positive = starboard). */
export function turningCircle(sim: Simulation, start: ManeuverStart, rudder: number, maxTime = 4000): TurningResult {
  startStraight(sim, start);
  const helm = rudder / sim.vessel.actuators.rudderMax;
  sim.controls = { ...sim.controls, helm };
  const track: TrackPoint[] = [];
  let psiPrev = 0;
  let psiUnwrapped = 0;
  const sgn = Math.sign(rudder) || 1;
  let advance = Number.NaN;
  let transfer = Number.NaN;
  let tactical = Number.NaN;
  let x180 = 0;
  let y180 = 0;
  let y360 = Number.NaN;
  let speed360 = Number.NaN;
  const steps = Math.ceil(maxTime / sim.dt);
  let prev = sample(sim, 0);
  track.push(prev);
  for (let i = 0; i < steps; i++) {
    sim.step();
    const s = sim.snapshot();
    psiUnwrapped += wrapPi(s.euler.yaw - psiPrev);
    psiPrev = s.euler.yaw;
    const cur = { t: s.t, x: s.position.x, y: s.position.y, heading: psiUnwrapped, rudder: s.actuators.rudder, speed: s.speedThroughWater };
    if (i % 4 === 0) track.push(cur);
    const turned = sgn * psiUnwrapped;
    const crossed = (target: number): number | null => {
      const a = sgn * prev.heading;
      if (a < target && turned >= target) return (target - a) / (turned - a);
      return null;
    };
    const f90 = crossed(Math.PI / 2);
    if (f90 !== null) {
      advance = prev.x + (cur.x - prev.x) * f90;
      transfer = Math.abs(prev.y + (cur.y - prev.y) * f90);
    }
    const f180 = crossed(Math.PI);
    if (f180 !== null) {
      x180 = prev.x + (cur.x - prev.x) * f180;
      y180 = prev.y + (cur.y - prev.y) * f180;
      tactical = Math.abs(y180);
    }
    const f360 = crossed(2 * Math.PI);
    if (f360 !== null) {
      y360 = prev.y + (cur.y - prev.y) * f360;
      speed360 = cur.speed;
    }
    prev = cur;
    if (turned > 2 * Math.PI + 0.1) break;
  }
  void x180;
  return {
    advance,
    transfer,
    tacticalDiameter: tactical,
    steadyDiameter: Math.abs(y180 - y360),
    speedLossRatio: speed360 / start.speed,
    track,
  };
}

export interface ZigZagResult {
  /** Overshoot angles (rad), first and second. */
  firstOvershoot: number;
  secondOvershoot: number;
  /** Time to reach the first check heading (s). */
  initialTurningTime: number;
  track: TrackPoint[];
}

/** Zig-zag δ/ψ manoeuvre (e.g. 10°/10°), first rudder execute to starboard when δ > 0. */
export function zigzag(sim: Simulation, start: ManeuverStart, rudder: number, checkHeading: number, maxTime = 2000): ZigZagResult {
  startStraight(sim, start);
  const rMax = sim.vessel.actuators.rudderMax;
  let side = Math.sign(rudder) || 1; // current rudder side
  const mag = Math.abs(rudder);
  sim.controls = { ...sim.controls, helm: (side * mag) / rMax };
  const track: TrackPoint[] = [];
  let psiPrev = 0;
  let psi = 0;
  let executes = 0;
  let ext1 = 0;
  let ext2 = 0;
  let tCheck = Number.NaN;
  const steps = Math.ceil(maxTime / sim.dt);
  for (let i = 0; i < steps; i++) {
    sim.step();
    const s = sim.snapshot();
    psi += wrapPi(s.euler.yaw - psiPrev);
    psiPrev = s.euler.yaw;
    if (i % 4 === 0) track.push({ t: s.t, x: s.position.x, y: s.position.y, heading: psi, rudder: s.actuators.rudder, speed: s.speedThroughWater });
    // switch the rudder when the heading passes the check angle on the current side
    if (side * psi >= checkHeading) {
      if (executes === 0) tCheck = s.t;
      side = -side;
      executes++;
      sim.controls = { ...sim.controls, helm: (side * mag) / rMax };
    }
    if (executes === 1) ext1 = Math.max(ext1, (Math.sign(rudder) || 1) * psi);
    if (executes === 2) ext2 = Math.max(ext2, -(Math.sign(rudder) || 1) * psi);
    if (executes >= 3) break;
  }
  return { firstOvershoot: ext1 - checkHeading, secondOvershoot: ext2 - checkHeading, initialTurningTime: tCheck, track };
}
