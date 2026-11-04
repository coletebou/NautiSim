import { describe, expect, it } from 'vitest';
import { calmWater } from '../src/core/environment';
import { createIntegrator } from '../src/core/integrators';
import { type BodyProperties, packState, STATE_SIZE, unpackState } from '../src/core/rigidbody';
import { ALL_DOF, dof6 } from '../src/core/types';
import { createPlaningBoat, RIB_65 } from '../src/hulls/planing';
import { quatFromEuler, quatIntegrate, quatMultiply, quatNormalize, quatSlerp, quatToEuler, rotate, rotateInverse } from '../src/math/quat';
import { approach, ittc57Cf, schoenherrCf, wrap2Pi, wrapPi } from '../src/math/scalar';
import { Simulation } from '../src/sim/simulation';
import { degToRad, knotsToMps, mpsToKnots, radToDeg } from '../src/units';

describe('units', () => {
  it('converts knots and degrees exactly', () => {
    expect(knotsToMps(1)).toBeCloseTo(0.514444, 6);
    expect(mpsToKnots(knotsToMps(15.5))).toBeCloseTo(15.5, 12);
    expect(radToDeg(degToRad(35))).toBeCloseTo(35, 12);
  });
});

describe('quaternions', () => {
  it('Euler round trip and rotation inverse', () => {
    const q = quatFromEuler(0.1, -0.2, 2.5);
    const e = quatToEuler(q);
    expect(e.roll).toBeCloseTo(0.1, 12);
    expect(e.pitch).toBeCloseTo(-0.2, 12);
    expect(e.yaw).toBeCloseTo(2.5, 12);
    const v = { x: 1, y: 2, z: 3 };
    const back = rotateInverse(q, rotate(q, v));
    expect(back.x).toBeCloseTo(1, 12);
    expect(back.y).toBeCloseTo(2, 12);
    expect(back.z).toBeCloseTo(3, 12);
  });

  it('yaw of 90° turns north into east (NED)', () => {
    const v = rotate(quatFromEuler(0, 0, Math.PI / 2), { x: 1, y: 0, z: 0 });
    expect(v.x).toBeCloseTo(0, 12);
    expect(v.y).toBeCloseTo(1, 12);
  });

  it('exponential-map integration is exact for constant rate', () => {
    let q = quatFromEuler(0, 0, 0);
    for (let i = 0; i < 1000; i++) q = quatIntegrate(q, { x: 0, y: 0, z: 0.3 }, 0.01);
    expect(quatToEuler(q).yaw).toBeCloseTo(3, 9);
    const n = Math.hypot(q.w, q.x, q.y, q.z);
    expect(n).toBeCloseTo(1, 12);
  });

  it('slerp hits the endpoints', () => {
    const a = quatFromEuler(0, 0, 0);
    const b = quatFromEuler(0, 0, 1);
    expect(quatToEuler(quatSlerp(a, b, 0.5)).yaw).toBeCloseTo(0.5, 9);
    const c = quatNormalize(quatMultiply(a, b));
    expect(quatToEuler(c).yaw).toBeCloseTo(1, 12);
  });
});

describe('scalar helpers', () => {
  it('wraps angles', () => {
    expect(wrapPi(3 * Math.PI)).toBeCloseTo(Math.PI, 12);
    expect(wrapPi(-3.5 * Math.PI)).toBeCloseTo(0.5 * Math.PI, 12);
    expect(wrap2Pi(-0.5)).toBeCloseTo(2 * Math.PI - 0.5, 12);
    expect(approach(0, 1, 0.25)).toBe(0.25);
    expect(approach(0.9, 1, 0.25)).toBe(1);
  });

  it('friction lines agree with tabulated values', () => {
    // ITTC-57 at Re = 1e7 is 0.00301; Schoenherr at 1e7 is about 0.00294
    expect(ittc57Cf(1e7)).toBeCloseTo(0.003, 4);
    expect(schoenherrCf(1e7)).toBeCloseTo(0.00293, 4);
    expect(schoenherrCf(1e9)).toBeLessThan(schoenherrCf(1e7));
  });
});

describe('rigid body integration', () => {
  const body: BodyProperties = {
    mass: 1000,
    inertia: { x: 100, y: 400, z: 400 },
    addedMass: dof6(),
    g: 0,
    mask: ALL_DOF,
  };

  for (const kind of ['rk4', 'semi-implicit-euler'] as const) {
    it(`${kind}: a free body spinning about its major axis keeps energy and momentum`, () => {
      const y = new Float64Array(STATE_SIZE);
      packState(y, { x: 0, y: 0, z: 0 }, quatFromEuler(0, 0, 0), { x: 2, y: 0, z: 0 }, { x: 0.01, y: 0, z: 1.0 });
      const it = createIntegrator(kind);
      const energy = (): number => {
        const s = unpackState(y);
        const w = s.omegaBody;
        return 0.5 * (100 * w.x * w.x + 400 * w.y * w.y + 400 * w.z * w.z) + 0.5 * 1000 * (s.velBody.x ** 2 + s.velBody.y ** 2 + s.velBody.z ** 2);
      };
      const e0 = energy();
      for (let i = 0; i < 1000; i++) it.step(y, i * 0.01, 0.01, body, () => {});
      expect(Math.abs(energy() - e0) / e0).toBeLessThan(kind === 'rk4' ? 1e-6 : 0.15);
      // linear momentum in the world frame is conserved: the CG moves in a straight line
      const s = unpackState(y);
      const vWorld = rotate(s.orientation, s.velBody);
      expect(Math.hypot(vWorld.x, vWorld.y, vWorld.z)).toBeCloseTo(2, kind === 'rk4' ? 6 : 0);
    });
  }

  it('heave oscillation of a floating hull has the hydrostatic natural period', () => {
    const v = createPlaningBoat(RIB_65);
    const sim = new Simulation(v, calmWater(), { integrator: 'rk4' });
    const s0 = sim.getState();
    sim.setState({ ...s0, position: { ...s0.position, z: s0.position.z + 0.03 } });
    // find successive upward zero crossings of the heave displacement
    const crossings: number[] = [];
    let prev = 0.03;
    for (let i = 0; i < Math.round(8 / sim.dt); i++) {
      sim.step();
      const dz = sim.snapshot().position.z - v.rest.z;
      if (prev > 0 && dz <= 0) crossings.push(sim.time);
      prev = dz;
    }
    expect(crossings.length).toBeGreaterThan(1);
    const period = (crossings[1] as number) - (crossings[0] as number);
    expect(period).toBeGreaterThan(0.6);
    expect(period).toBeLessThan(2.5);
  });
});

describe('determinism', () => {
  it('two simulations with the same inputs are bit-identical', () => {
    const run = (): number[] => {
      const v = createPlaningBoat(RIB_65);
      const sim = new Simulation(v, calmWater({ wind: { x: 3, y: 4, z: 0 }, current: { x: 0.5, y: 0, z: 0 } }));
      const out: number[] = [];
      for (let i = 0; i < 600; i++) {
        sim.controls = { throttle: Math.min(1, i / 200), helm: Math.sin(i / 50), thruster: 0 };
        sim.step();
      }
      const s = sim.getState();
      out.push(s.position.x, s.position.y, s.position.z, s.orientation.w, s.orientation.x, s.velBody.x, s.omegaBody.z);
      return out;
    };
    expect(run()).toEqual(run());
  });
});
