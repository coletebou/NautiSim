import { describe, expect, it } from 'vitest';
import { savitskyCL0, savitskyCLBeta, savitskyCpFraction, savitskyEquilibrium, savitskyEvaluate } from '../src/forces/savitsky';
import { KNOT_IN_MPS } from '../src/units';
import { SAVITSKY_EXAMPLE as EX } from '../src/validation/references';

const rel = (a: number, b: number): number => Math.abs(a - b) / Math.abs(b);
const G = 9.80665;

describe('validation: Savitsky (1964) planing equations against a published worked example', () => {
  const V = EX.speedKnots * KNOT_IN_MPS;
  const hull = { beam: EX.beam, deadriseDeg: EX.deadriseDeg, weight: EX.massTonnes * 1000 * G, deltaCf: EX.deltaCf };
  const p = savitskyEvaluate(hull, V, EX.trimDeg, EX.lambda, EX.rho, EX.nu, G);

  it('lift coefficients and lift', () => {
    expect(p.cv).toBeCloseTo(EX.expected.cv, 3);
    expect(rel(p.cl0, EX.expected.cl0)).toBeLessThan(1e-4);
    expect(rel(p.clBeta, EX.expected.clBeta)).toBeLessThan(1e-4);
    expect(rel(p.lift, EX.expected.liftN)).toBeLessThan(1e-3);
    // the example was set up so that lift equals the weight
    expect(rel(p.lift, hull.weight)).toBeLessThan(1e-3);
  });

  it('mean bottom velocity, friction and pressure drag', () => {
    expect(rel(p.meanVelocity, EX.expected.meanVelocity)).toBeLessThan(1e-3);
    expect(rel(p.cf, EX.expected.cf)).toBeLessThan(2e-3);
    // the calculator evaluates ½ρV²SCf with the forward speed; Savitsky (1964) uses the mean bottom
    // velocity V_m, which we follow, so scale its friction by (V_m / V)²
    const vmRatio = EX.expected.meanVelocity / V;
    expect(rel(p.friction, EX.expected.frictionN * vmRatio * vmRatio)).toBeLessThan(2e-3);
    expect(rel(p.pressureDrag, EX.expected.pressureDragN)).toBeLessThan(2e-3);
  });

  it('total resistance follows Savitsky D = Δ tanτ + Df / cosτ and is within 2 % of the example', () => {
    const tau = (EX.trimDeg * Math.PI) / 180;
    expect(rel(p.resistance, hull.weight * Math.tan(tau) + p.friction / Math.cos(tau))).toBeLessThan(1e-9);
    expect(rel(p.resistance, EX.expected.totalN)).toBeLessThan(0.02);
  });
});

describe('Savitsky equation properties', () => {
  it('deadrise reduces lift and the centre of pressure moves aft with speed', () => {
    const cl0 = savitskyCL0(4, 2, 3);
    expect(savitskyCLBeta(cl0, 0)).toBeCloseTo(cl0, 12);
    expect(savitskyCLBeta(cl0, 20)).toBeLessThan(cl0);
    expect(savitskyCpFraction(6, 2)).toBeGreaterThan(savitskyCpFraction(2, 2));
    expect(savitskyCpFraction(20, 2)).toBeCloseTo(0.75, 2);
  });

  it('equilibrium solver balances lift with weight and puts the CP at the LCG', () => {
    const hull = { beam: 2.0, deadriseDeg: 22, weight: 1600 * G, lcg: 2.4, deltaCf: 0.0004 };
    for (const kn of [25, 35, 45]) {
      const V = kn * KNOT_IN_MPS;
      const eq = savitskyEquilibrium(hull, V, 1025, 1.19e-6, G);
      expect(rel(eq.lift, hull.weight)).toBeLessThan(1e-4);
      expect(Math.abs(eq.cp - hull.lcg)).toBeLessThan(1e-3);
      expect(eq.tauDeg).toBeGreaterThan(1);
      expect(eq.tauDeg).toBeLessThan(8);
    }
    // running trim falls as speed rises past the hump
    const t25 = savitskyEquilibrium(hull, 25 * KNOT_IN_MPS, 1025, 1.19e-6, G).tauDeg;
    const t45 = savitskyEquilibrium(hull, 45 * KNOT_IN_MPS, 1025, 1.19e-6, G).tauDeg;
    expect(t45).toBeLessThan(t25);
  });
});
