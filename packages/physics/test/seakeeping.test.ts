import { presetEnvironment } from '@nautisim/environment';
import { describe, expect, it } from 'vitest';
import { createKVLCC2 } from '../src/hulls/kvlcc2';
import { createPlaningBoat, RIB_65 } from '../src/hulls/planing';
import { Simulation } from '../src/sim/simulation';

const DEG = Math.PI / 180;

describe('seakeeping in irregular seas (environment package)', () => {
  it('a RIB at rest in a moderate sea rides the waves and stays upright', () => {
    const env = presetEnvironment('moderate', { seed: 3, waveComponents: 16 });
    const sim = new Simulation(createPlaningBoat(RIB_65), env);
    let zMin = Infinity;
    let zMax = -Infinity;
    let maxRoll = 0;
    for (let i = 0; i < Math.round(60 / sim.dt); i++) {
      sim.step();
      const s = sim.snapshot();
      if (s.t > 10) {
        zMin = Math.min(zMin, s.position.z);
        zMax = Math.max(zMax, s.position.z);
        maxRoll = Math.max(maxRoll, Math.abs(s.euler.roll));
      }
    }
    // heaves with roughly the wave height (Hs = 1.8 m)
    expect(zMax - zMin).toBeGreaterThan(0.8);
    expect(zMax - zMin).toBeLessThan(5);
    expect(maxRoll).toBeLessThan(45 * DEG);
  }, 60_000);

  for (const sea of ['slight', 'moderate'] as const) {
    it(`a RIB at speed in a ${sea} sea bounces but never stands on its tail or flips`, () => {
      const env = presetEnvironment(sea, { seed: 7, fromDirection: (40 * Math.PI) / 180, waveComponents: 16 });
      const sim = new Simulation(createPlaningBoat(RIB_65), env);
      sim.controls = { throttle: 0.75, helm: 0, thruster: 0 };
      let maxTrim = 0;
      let maxRoll = 0;
      for (let i = 0; i < Math.round(40 / sim.dt); i++) {
        if (i === Math.round(15 / sim.dt)) sim.controls = { throttle: 0.75, helm: 0.3, thruster: 0 };
        sim.step();
        const s = sim.snapshot();
        maxTrim = Math.max(maxTrim, s.euler.pitch);
        maxRoll = Math.max(maxRoll, Math.abs(s.euler.roll));
      }
      expect(maxTrim).toBeLessThan(25 * DEG);
      expect(maxRoll).toBeLessThan(35 * DEG);
      expect(sim.snapshot().speedThroughWater / 0.5144).toBeGreaterThan(15);
    }, 60_000);
  }

  it('a full-scale tanker barely notices a moderate sea', () => {
    const env = presetEnvironment('moderate', { seed: 5, waveComponents: 16 });
    const v = createKVLCC2({ scale: 1 });
    const sim = new Simulation(v, env);
    sim.reset({ speed: v.approachSpeed });
    sim.controls = { throttle: v.approachRps / v.actuators.rpsMax, helm: 0, thruster: 0 };
    sim.overrideActuators({ rps: v.approachRps });
    let maxRoll = 0;
    let maxPitch = 0;
    for (let i = 0; i < Math.round(120 / sim.dt); i++) {
      sim.step();
      const s = sim.snapshot();
      maxRoll = Math.max(maxRoll, Math.abs(s.euler.roll));
      maxPitch = Math.max(maxPitch, Math.abs(s.euler.pitch));
    }
    expect(maxRoll).toBeLessThan(3 * DEG);
    expect(maxPitch).toBeLessThan(1 * DEG);
    expect(sim.snapshot().speedThroughWater).toBeGreaterThan(0.85 * v.approachSpeed);
  }, 60_000);
});
