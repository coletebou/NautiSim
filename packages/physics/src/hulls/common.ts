import { calmWater } from '../core/environment';
import type { Dof6 } from '../core/types';
import type { TriangleMesh } from '../geometry/mesh';
import { stabilityProperties } from '../hydrostatics/equilibrium';
import { computeHydrostatics, createHydroScratch, newHydroResult } from '../hydrostatics/hydrostatics';
import { quatFromEuler, rotateInverse } from '../math/quat';

export interface RestoringEstimate {
  rest: { z: number; pitch: number; roll: number };
  volume: number;
  gm: number;
  waterplaneArea: number;
  /** Heave, roll and pitch stiffness (N/m, N·m/rad). */
  kHeave: number;
  kRoll: number;
  kPitch: number;
}

/** Calm-water equilibrium and linearised restoring stiffness of a hull mesh. */
export function estimateRestoring(mesh: TriangleMesh, mass: number, rho: number, g: number): RestoringEstimate {
  const props = stabilityProperties(mesh, mass, { rho, g, solvePitch: true });
  const eq = props.equilibrium;
  const env = calmWater({ rho, g });
  const scratch = createHydroScratch(mesh);
  const res = newHydroResult();
  const pitchMoment = (th: number): number => {
    const q = quatFromEuler(0, eq.pitch + th, 0);
    computeHydrostatics(mesh, { x: 0, y: 0, z: eq.z }, q, env, 0, scratch, res);
    return rotateInverse(q, res.moment).y;
  };
  const dth = 0.2 * (Math.PI / 180);
  const kPitch = -(pitchMoment(dth) - pitchMoment(-dth)) / (2 * dth);
  return {
    rest: { z: eq.z, pitch: eq.pitch, roll: 0 },
    volume: eq.volume,
    gm: props.gm,
    waterplaneArea: props.waterplaneArea,
    kHeave: rho * g * props.waterplaneArea,
    kRoll: mass * g * Math.max(props.gm, 0.01),
    kPitch: Math.max(kPitch, 1),
  };
}

/** Linear damping coefficients from target damping ratios: c = 2 ζ √(k (M + A)). */
export function dampingFromRatios(
  r: RestoringEstimate,
  mass: number,
  inertia: { x: number; y: number },
  added: Pick<Dof6, 'heave' | 'roll' | 'pitch'>,
  zeta: { heave: number; roll: number; pitch: number },
): Pick<Dof6, 'heave' | 'roll' | 'pitch'> {
  return {
    heave: 2 * zeta.heave * Math.sqrt(r.kHeave * (mass + added.heave)),
    roll: 2 * zeta.roll * Math.sqrt(r.kRoll * (inertia.x + added.roll)),
    pitch: 2 * zeta.pitch * Math.sqrt(r.kPitch * (inertia.y + added.pitch)),
  };
}
