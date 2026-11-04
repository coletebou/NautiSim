import type { Vec3 } from '../math/vec3';
import { GRAVITY, NU_SEAWATER_15C, RHO_AIR, RHO_SEAWATER } from '../units';

/**
 * What the physics needs to know about the world. All positions and vectors are in the NED world
 * frame (x north, y east, z down). `@nautisim/environment` provides a full implementation; physics
 * only depends on this interface.
 */
export interface EnvironmentSampler {
  readonly rho: number;
  readonly nu: number;
  readonly g: number;
  readonly rhoAir: number;
  /** Free-surface elevation η (m, positive up) at a horizontal position and time. */
  elevation(x: number, y: number, t: number): number;
  /** Water particle velocity (current + wave orbital) at a point, NED (m/s). Writes into out. */
  waterVelocity(x: number, y: number, z: number, t: number, out: Vec3): Vec3;
  /** Wind velocity (the direction the air moves toward) at 10 m height, NED (m/s). Writes into out. */
  wind(x: number, y: number, t: number, out: Vec3): Vec3;
  /** Water depth below the mean surface (m); Infinity for deep water. */
  depth(x: number, y: number): number;
}

export interface CalmWaterOptions {
  rho?: number;
  nu?: number;
  g?: number;
  rhoAir?: number;
  /** Uniform current, NED (m/s). */
  current?: Readonly<Vec3>;
  /** Uniform wind, NED (m/s, direction the air moves toward). */
  wind?: Readonly<Vec3>;
  depth?: number;
}

/** Flat water with optional uniform current and wind. Used by tests and validation runs. */
export function calmWater(opts: CalmWaterOptions = {}): EnvironmentSampler {
  const cur = opts.current ?? { x: 0, y: 0, z: 0 };
  const wnd = opts.wind ?? { x: 0, y: 0, z: 0 };
  const depth = opts.depth ?? Number.POSITIVE_INFINITY;
  return {
    rho: opts.rho ?? RHO_SEAWATER,
    nu: opts.nu ?? NU_SEAWATER_15C,
    g: opts.g ?? GRAVITY,
    rhoAir: opts.rhoAir ?? RHO_AIR,
    elevation: () => 0,
    waterVelocity: (_x, _y, _z, _t, out) => {
      out.x = cur.x;
      out.y = cur.y;
      out.z = 0;
      return out;
    },
    wind: (_x, _y, _t, out) => {
      out.x = wnd.x;
      out.y = wnd.y;
      out.z = 0;
      return out;
    },
    depth: () => depth,
  };
}
