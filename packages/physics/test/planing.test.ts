import { beforeAll, describe, expect, it } from 'vitest';
import { calmWater } from '../src/core/environment';
import { createPlaningBoat, type PlaningVessel, RIB_65, RUNABOUT_8 } from '../src/hulls/planing';
import { Simulation } from '../src/sim/simulation';
import { KNOT_IN_MPS } from '../src/units';

const DEG = Math.PI / 180;

function runStraight(v: PlaningVessel, throttle: number, seconds: number) {
  const sim = new Simulation(v, calmWater());
  sim.controls = { throttle, helm: 0, thruster: 0 };
  sim.run(Math.round((seconds - 10) / sim.dt));
  const pitches: number[] = [];
  for (let i = 0; i < 10; i++) {
    sim.run(Math.round(1 / sim.dt));
    pitches.push(sim.snapshot().euler.pitch);
  }
  const s = sim.snapshot();
  const mean = pitches.reduce((a, b) => a + b, 0) / pitches.length;
  const spread = Math.max(...pitches) - Math.min(...pitches);
  return { snap: s, trim: mean, trimSpread: spread };
}

for (const def of [RIB_65, RUNABOUT_8]) {
  describe(`planing boat: ${def.name}`, () => {
    let vessel: PlaningVessel;
    beforeAll(() => {
      vessel = createPlaningBoat(def);
    });

    it('floats level at rest with positive stability', () => {
      expect(Math.abs(vessel.rest.pitch)).toBeLessThan(1.5 * DEG);
      expect(vessel.meta.gm as number).toBeGreaterThan(0.3);
      const sim = new Simulation(vessel, calmWater());
      sim.run(Math.round(5 / sim.dt));
      const s = sim.snapshot();
      expect(Math.abs(s.position.z - vessel.rest.z)).toBeLessThan(0.01);
      expect(s.speedThroughWater).toBeLessThan(0.05);
    });

    it('full throttle: gets on the plane at a realistic top speed and trim, without porpoising', () => {
      const r = runStraight(vessel, 1, 60);
      const kn = r.snap.speedThroughWater / KNOT_IN_MPS;
      expect(kn).toBeGreaterThan(33);
      expect(kn).toBeLessThan(52);
      expect(r.trim).toBeGreaterThan(1 * DEG);
      expect(r.trim).toBeLessThan(6 * DEG);
      expect(r.trimSpread).toBeLessThan(0.5 * DEG);
      // dynamic lift carries a large share of the weight on the plane
      expect((r.snap.diag['planing.lift'] ?? 0) / (vessel.mass * 9.81)).toBeGreaterThan(0.3);
      // the engine is power-limited near its rated rpm
      expect(r.snap.engineRpm).toBeGreaterThan(0.85 * def.engine.ratedRpm);
      expect(r.snap.engineRpm).toBeLessThan(1.05 * def.engine.ratedRpm);
    }, 30_000);

    it('idle speed is a slow displacement crawl', () => {
      const r = runStraight(vessel, 0.15, 40);
      const kn = r.snap.speedThroughWater / KNOT_IN_MPS;
      expect(kn).toBeGreaterThan(2);
      expect(kn).toBeLessThan(7);
      expect(Math.abs(r.trim)).toBeLessThan(2 * DEG);
    }, 30_000);

    it('hard-over turn on the plane stays upright and settles', () => {
      const sim = new Simulation(vessel, calmWater());
      sim.controls = { throttle: 0.7, helm: 0, thruster: 0 };
      sim.run(Math.round(30 / sim.dt));
      sim.controls = { throttle: 0.7, helm: 1, thruster: 0 };
      let maxRoll = 0;
      for (let i = 0; i < Math.round(15 / sim.dt); i++) {
        sim.step();
        maxRoll = Math.max(maxRoll, Math.abs(sim.snapshot().euler.roll));
      }
      const s = sim.snapshot();
      expect(maxRoll).toBeLessThan(25 * DEG);
      expect(s.omegaBody.z).toBeGreaterThan(0.1); // turning to starboard
      const radius = s.speedThroughWater / s.omegaBody.z;
      expect(radius / def.length).toBeGreaterThan(2);
      expect(radius / def.length).toBeLessThan(15);
    }, 30_000);
  });
}
