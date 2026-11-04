import { describe, expect, it } from 'vitest';
import { calmWater } from '../src/core/environment';
import { loftStations } from '../src/geometry/loft';
import { boxMesh, meshVolumeAndCentroid, translateMesh } from '../src/geometry/mesh';
import { createShipHull } from '../src/geometry/shipHull';
import { solveEquilibrium, stabilityProperties } from '../src/hydrostatics/equilibrium';
import { computeHydrostatics, createHydroScratch, newHydroResult } from '../src/hydrostatics/hydrostatics';
import { quatFromEuler, quatIdentity } from '../src/math/quat';

const RHO = 1025;
const G = 9.81;
const env = calmWater({ rho: RHO, g: G });

describe('mesh basics', () => {
  it('box volume and centroid are exact', () => {
    const m = translateMesh(boxMesh(10, 4, 3), { x: 1, y: 2, z: 3 });
    const { volume, centroid } = meshVolumeAndCentroid(m);
    expect(volume).toBeCloseTo(120, 9);
    expect(centroid.x).toBeCloseTo(1, 9);
    expect(centroid.y).toBeCloseTo(2, 9);
    expect(centroid.z).toBeCloseTo(3, 9);
  });

  it('lofted box matches the box mesh', () => {
    const ring: Array<[number, number]> = [
      [2, -1.5],
      [2, 1.5],
      [-2, 1.5],
      [-2, -1.5],
    ];
    const m = loftStations([
      { x: -5, ring },
      { x: 0, ring },
      { x: 5, ring },
    ]);
    expect(meshVolumeAndCentroid(m).volume).toBeCloseTo(120, 9);
  });
});

describe('hydrostatics (validation: box barge, closed form)', () => {
  const L = 10;
  const B = 4;
  const H = 3;
  const mesh = boxMesh(L, B, H); // CG at the box centre, KG = H/2

  it('buoyancy equals ρgV of the immersed box at any draft', () => {
    const scratch = createHydroScratch(mesh);
    const res = newHydroResult();
    for (const T of [0.3, 1.0, 1.5, 2.7]) {
      // keel at body z = +1.5 → world z of the CG is T − 1.5
      computeHydrostatics(mesh, { x: 0, y: 0, z: T - H / 2 }, quatIdentity(), env, 0, scratch, res);
      expect(res.volume).toBeCloseTo(L * B * T, 6);
      expect(res.force.x).toBeCloseTo(0, 6);
      expect(res.force.y).toBeCloseTo(0, 6);
      // centre of buoyancy straight below the CG → no moment
      expect(Math.hypot(res.moment.x, res.moment.y, res.moment.z)).toBeLessThan(1e-6 * RHO * G * L * B * T);
      // wetted area = bottom + sides
      expect(res.wettedArea).toBeCloseTo(L * B + 2 * (L + B) * T, 6);
    }
  });

  it('equilibrium draft is m/(ρLB)', () => {
    const T = 1.2;
    const mass = RHO * L * B * T;
    const eq = solveEquilibrium(mesh, mass, { rho: RHO, g: G, solvePitch: true });
    expect(eq.z + H / 2).toBeCloseTo(T, 6);
    expect(eq.pitch).toBeCloseTo(0, 8);
  });

  it('metacentric height matches KB + BM − KG and the wall-sided GZ formula', () => {
    const T = 1.5;
    const mass = RHO * L * B * T;
    const props = stabilityProperties(mesh, mass, { rho: RHO, g: G });
    const KB = T / 2;
    const BM = (B * B) / (12 * T);
    const KG = H / 2;
    const GM = KB + BM - KG;
    expect(props.gm).toBeCloseTo(GM, 4);
    expect(props.waterplaneArea).toBeCloseTo(L * B, 4);
    for (const phiDeg of [5, 10, 15]) {
      const phi = (phiDeg * Math.PI) / 180;
      const wallSided = Math.sin(phi) * (GM + 0.5 * BM * Math.tan(phi) ** 2);
      expect(props.gz(phi)).toBeCloseTo(wallSided, 6);
    }
  });

  it('a heeled box produces a righting moment in the right sense', () => {
    const T = 1.5;
    const scratch = createHydroScratch(mesh);
    const res = newHydroResult();
    const heel = 0.1; // starboard down
    computeHydrostatics(mesh, { x: 0, y: 0, z: T - H / 2 }, quatFromEuler(heel, 0, 0), env, 0, scratch, res);
    // GM > 0 for this box → moment about +x is negative (rolls back to port)
    expect(res.moment.x).toBeLessThan(0);
  });
});

describe('parametric ship hull', () => {
  it('hits the target block coefficient and LCB', () => {
    const p = { length: 7, beam: 1.27, draft: 0.46, depth: 0.65, blockCoefficient: 0.81, lcb: 0.24 };
    const { mesh } = createShipHull({ ...p, stations: 81, pointsBelow: 16 });
    const scratch = createHydroScratch(mesh);
    const res = newHydroResult();
    // mesh origin at midship on the waterline; place it at the surface
    computeHydrostatics(mesh, { x: 0, y: 0, z: 0 }, quatIdentity(), env, 0, scratch, res);
    const cb = res.volume / (p.length * p.beam * p.draft);
    expect(cb).toBeCloseTo(0.81, 2);
    // centre of buoyancy from the moment about the origin: My = −Fz·xB … check sign via a lever
    const xB = res.moment.y / (RHO * G * res.volume);
    expect(xB).toBeCloseTo(p.lcb, 2);
  });
});
