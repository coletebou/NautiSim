import { deriveSeed } from './random';
import { G } from './spectrum';
import { type SeaState, WaveField } from './waves';
import { WindField } from './wind';

export interface Vec3Like {
  x: number;
  y: number;
  z: number;
}

export interface CurrentOptions {
  /** Speed (m/s) and the direction it flows TOWARD (compass radians), oceanographic convention. */
  speed: number;
  toDirection: number;
  /** Tidal modulation: amplitude as a fraction of the speed and period (s). */
  tidalFraction?: number;
  tidalPeriod?: number;
}

export type DepthModel = { kind: 'constant'; depth: number } | { kind: 'shoal'; deep: number; shallow: number; distance: number; bearing: number };

export interface EnvironmentOptions {
  seed: number;
  sea: SeaState;
  wind: { speed: number; fromDirection: number; turbulence?: number };
  current?: CurrentOptions;
  depth?: DepthModel;
  waveComponents?: number;
  rho?: number;
  nu?: number;
  rhoAir?: number;
  g?: number;
}

/**
 * The full marine environment. Implements the physics package's EnvironmentSampler structurally
 * (rho, nu, g, rhoAir, elevation, waterVelocity, wind, depth) without importing it.
 */
export class Environment {
  readonly rho: number;
  readonly nu: number;
  readonly g: number;
  readonly rhoAir: number;
  readonly waves: WaveField;
  readonly windField: WindField;
  readonly options: Readonly<EnvironmentOptions>;
  private readonly orbital: Vec3Like = { x: 0, y: 0, z: 0 };

  constructor(o: EnvironmentOptions) {
    this.options = o;
    this.rho = o.rho ?? 1025;
    this.nu = o.nu ?? 1.188e-6;
    this.g = o.g ?? G;
    this.rhoAir = o.rhoAir ?? 1.225;
    const depth0 = o.depth?.kind === 'constant' ? o.depth.depth : Number.POSITIVE_INFINITY;
    this.waves = new WaveField(o.sea, { seed: deriveSeed(o.seed, 'waves'), components: o.waveComponents ?? 24, depth: depth0, g: this.g });
    this.windField = new WindField({ ...o.wind, seed: deriveSeed(o.seed, 'wind') });
  }

  elevation(x: number, y: number, t: number): number {
    return this.waves.elevation(x, y, t);
  }

  waterVelocity(x: number, y: number, z: number, t: number, out: Vec3Like): Vec3Like {
    this.waves.orbitalVelocity(x, y, z, t, this.orbital);
    const c = this.options.current;
    let cx = 0;
    let cy = 0;
    if (c && c.speed > 0) {
      const tide = 1 + (c.tidalFraction ?? 0) * Math.sin((2 * Math.PI * t) / (c.tidalPeriod ?? 44_712));
      cx = c.speed * tide * Math.cos(c.toDirection);
      cy = c.speed * tide * Math.sin(c.toDirection);
    }
    out.x = this.orbital.x + cx;
    out.y = this.orbital.y + cy;
    out.z = this.orbital.z;
    return out;
  }

  wind(_x: number, _y: number, t: number, out: Vec3Like): Vec3Like {
    return this.windField.velocity(t, out);
  }

  depth(x: number, y: number): number {
    const d = this.options.depth;
    if (!d) return Number.POSITIVE_INFINITY;
    if (d.kind === 'constant') return d.depth;
    // linear shoal toward `bearing`, reaching `shallow` at `distance` metres from the origin
    const along = x * Math.cos(d.bearing) + y * Math.sin(d.bearing);
    const f = Math.min(1, Math.max(0, along / d.distance));
    return d.deep + (d.shallow - d.deep) * f;
  }
}

/** Named sea states, roughly following the Douglas sea scale with matching Beaufort winds. */
export const SEA_STATES = {
  calm: { label: 'Calm (glassy)', hs: 0, tp: 4, wind: 1 },
  smooth: { label: 'Smooth (Douglas 1)', hs: 0.1, tp: 2.5, wind: 3 },
  slight: { label: 'Slight (Douglas 3)', hs: 0.8, tp: 4.5, wind: 7 },
  moderate: { label: 'Moderate (Douglas 4)', hs: 1.8, tp: 6.5, wind: 10 },
  rough: { label: 'Rough (Douglas 5)', hs: 3.0, tp: 8.5, wind: 15 },
  veryRough: { label: 'Very rough (Douglas 6)', hs: 5.0, tp: 10.5, wind: 20 },
} as const;

export type SeaStateId = keyof typeof SEA_STATES;

export function presetEnvironment(id: SeaStateId, opts: { seed?: number; fromDirection?: number; waveComponents?: number } = {}): Environment {
  const p = SEA_STATES[id];
  const from = opts.fromDirection ?? Math.PI / 4;
  return new Environment({
    seed: opts.seed ?? 1,
    sea: { hs: p.hs, tp: p.tp, fromDirection: from, spreading: 4 },
    wind: { speed: p.wind, fromDirection: from, turbulence: 0.1 },
    ...(opts.waveComponents !== undefined && { waveComponents: opts.waveComponents }),
  });
}
