import { beforeAll, describe, expect, it } from 'vitest';
import { calmWater } from '../src/core/environment';
import { createKVLCC2, type KVLCC2Vessel } from '../src/hulls/kvlcc2';
import { Simulation } from '../src/sim/simulation';
import { DEG_IN_RAD } from '../src/units';
import { turningCircle, type TurningResult, zigzag, type ZigZagResult } from '../src/validation/maneuvers';
import { KVLCC2_L7_REFERENCE as REF } from '../src/validation/references';

/**
 * Validation against the KVLCC2 L7 benchmark. We implement the same MMG equations as
 * Yasukawa & Yoshimura, so the primary check is closeness to their computed values; the secondary
 * check is the MARIN free-running band. Tolerances follow the literature's own scatter: about
 * ±0.1 L on turning geometry and a few degrees on overshoot angles.
 */
const pct = (a: number, b: number): number => Math.abs(a - b) / b;
const deg = (r: number): number => r / DEG_IN_RAD;

describe('validation: KVLCC2 MMG manoeuvres (L7 model, Fn 0.142)', () => {
  let vessel: KVLCC2Vessel;
  let sim: Simulation;
  const turns: Record<string, TurningResult> = {};
  const zz: Record<string, ZigZagResult> = {};

  beforeAll(() => {
    vessel = createKVLCC2({ scale: 45.714, dof: 'planar' });
    sim = new Simulation(vessel, calmWater({ rho: 1025, nu: 1.0e-6, g: 9.81 }), { fixedStep: 0.05 });
    const start = { speed: REF.approachSpeed, rps: REF.selfPropulsionRps };
    turns.starboard = turningCircle(sim, start, 35 * DEG_IN_RAD);
    turns.port = turningCircle(sim, start, -35 * DEG_IN_RAD);
    zz['10/10'] = zigzag(sim, start, 10 * DEG_IN_RAD, 10 * DEG_IN_RAD);
    zz['-10/-10'] = zigzag(sim, start, -10 * DEG_IN_RAD, 10 * DEG_IN_RAD);
    zz['20/20'] = zigzag(sim, start, 20 * DEG_IN_RAD, 20 * DEG_IN_RAD);
    zz['-20/-20'] = zigzag(sim, start, -20 * DEG_IN_RAD, 20 * DEG_IN_RAD);
  }, 120_000);

  it('model matches the published L7 particulars', () => {
    expect(vessel.dimensions.length).toBeCloseTo(7.0, 3);
    expect(vessel.mass / 1025).toBeCloseTo(3.27, 2);
    expect(vessel.approachSpeed).toBeCloseTo(REF.approachSpeed, 3);
  });

  for (const side of ['starboard', 'port'] as const) {
    it(`35° turn to ${side}: advance and tactical diameter within 5 % of Yasukawa & Yoshimura`, () => {
      const L = vessel.dimensions.length;
      const r = turns[side] as TurningResult;
      const ref = REF.turning[side];
      expect(pct(r.advance / L, ref.mmg.advance)).toBeLessThan(0.05);
      expect(pct(r.tacticalDiameter / L, ref.mmg.tacticalDiameter)).toBeLessThan(0.05);
    });
    it(`35° turn to ${side}: inside 8 % of the MARIN free-running result`, () => {
      const L = vessel.dimensions.length;
      const r = turns[side] as TurningResult;
      const ref = REF.turning[side];
      expect(pct(r.advance / L, ref.marin.advance)).toBeLessThan(0.08);
      expect(pct(r.tacticalDiameter / L, ref.marin.tacticalDiameter)).toBeLessThan(0.08);
    });
  }

  it('turning is roughly symmetric and loses speed like a full-form tanker', () => {
    const s = turns.starboard as TurningResult;
    const p = turns.port as TurningResult;
    expect(pct(s.tacticalDiameter, p.tacticalDiameter)).toBeLessThan(0.05);
    expect(s.speedLossRatio).toBeGreaterThan(0.2);
    expect(s.speedLossRatio).toBeLessThan(0.5);
  });

  for (const key of ['10/10', '-10/-10', '20/20', '-20/-20'] as const) {
    it(`${key} zig-zag overshoots within 2.5° / 3.5° of Yasukawa & Yoshimura`, () => {
      const r = zz[key] as ZigZagResult;
      const [os1, os2] = REF.zigzag[key].mmg;
      expect(Math.abs(deg(r.firstOvershoot) - os1)).toBeLessThan(2.5);
      expect(Math.abs(deg(r.secondOvershoot) - os2)).toBeLessThan(3.5);
    });
    it(`${key} zig-zag overshoots within 7.5° of MARIN free-running`, () => {
      const r = zz[key] as ZigZagResult;
      const [os1, os2] = REF.zigzag[key].marin;
      expect(Math.abs(deg(r.firstOvershoot) - os1)).toBeLessThan(7.5);
      expect(Math.abs(deg(r.secondOvershoot) - os2)).toBeLessThan(7.5);
    });
  }

  it('6-DOF run (free heave, roll, pitch) gives the same turning circle as the planar run', () => {
    const v6 = createKVLCC2({ scale: 45.714, dof: '6dof' });
    const s6 = new Simulation(v6, calmWater({ rho: 1025, nu: 1.0e-6, g: 9.81 }), { fixedStep: 0.05 });
    const r6 = turningCircle(s6, { speed: REF.approachSpeed, rps: REF.selfPropulsionRps }, 35 * DEG_IN_RAD);
    const r = turns.starboard as TurningResult;
    expect(pct(r6.tacticalDiameter, r.tacticalDiameter)).toBeLessThan(0.02);
    // a tanker heels outward only a little in a turn
    const snap = s6.snapshot();
    expect(Math.abs(snap.euler.roll)).toBeLessThan(3 * DEG_IN_RAD);
  }, 60_000);

  it('full-scale self-propulsion holds 15.5 kn in a straight line', () => {
    const full = createKVLCC2({ scale: 1, dof: 'planar' });
    const s = new Simulation(full, calmWater(), {});
    s.reset({ speed: full.approachSpeed });
    s.controls = { throttle: full.approachRps / full.actuators.rpsMax, helm: 0, thruster: 0 };
    s.overrideActuators({ rps: full.approachRps });
    s.run(Math.round(300 / s.dt));
    expect(pct(s.snapshot().speedThroughWater, full.approachSpeed)).toBeLessThan(0.01);
    // propeller of a VLCC turns at around 75–80 rpm at service speed
    expect(full.approachRps * 60).toBeGreaterThan(55);
    expect(full.approachRps * 60).toBeLessThan(95);
  }, 60_000);
});
