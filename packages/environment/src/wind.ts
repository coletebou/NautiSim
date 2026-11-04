import { createRng } from './random';

export interface WindOptions {
  /** Mean speed at 10 m (m/s). */
  speed: number;
  /** Direction the wind blows FROM, compass radians. */
  fromDirection: number;
  /** Turbulence intensity σ_U / U (0.1 at sea is typical). */
  turbulence?: number;
  /** Slow direction wander amplitude (rad). */
  directionWander?: number;
  seed: number;
}

/**
 * Mean wind plus deterministic gusts. The gust signal is a sum of sinusoids whose amplitudes
 * follow a Davenport-like spectral slope (more energy in long periods), with seeded phases. It is a
 * pure function of time, so physics, replays and tests all see the same gusts.
 */
export class WindField {
  readonly opts: Readonly<WindOptions>;
  private readonly amp: Float64Array;
  private readonly freq: Float64Array;
  private readonly phase: Float64Array;
  private readonly dirAmp: Float64Array;
  private readonly dirPhase: Float64Array;

  constructor(opts: WindOptions) {
    this.opts = { ...opts };
    const rng = createRng(opts.seed);
    const n = 12;
    this.amp = new Float64Array(n);
    this.freq = new Float64Array(n);
    this.phase = new Float64Array(n);
    this.dirAmp = new Float64Array(3);
    this.dirPhase = new Float64Array(3);
    const sigma = (opts.turbulence ?? 0.1) * opts.speed;
    let sum = 0;
    for (let i = 0; i < n; i++) {
      // periods from about 8 s to 10 min, log-spaced
      const T = 8 * Math.exp((Math.log(600 / 8) * i) / (n - 1)) * rng.range(0.85, 1.15);
      this.freq[i] = (2 * Math.PI) / T;
      const a = T ** (1 / 3);
      this.amp[i] = a;
      sum += 0.5 * a * a;
      this.phase[i] = rng.range(0, 2 * Math.PI);
    }
    const k = sum > 0 ? sigma / Math.sqrt(sum) : 0;
    for (let i = 0; i < n; i++) this.amp[i] = (this.amp[i] as number) * k;
    for (let i = 0; i < 3; i++) {
      this.dirAmp[i] = (opts.directionWander ?? 0.08) / (i + 1);
      this.dirPhase[i] = rng.range(0, 2 * Math.PI);
    }
  }

  /** Wind speed (m/s) at time t. */
  speedAt(t: number): number {
    let u = this.opts.speed;
    for (let i = 0; i < this.amp.length; i++) u += (this.amp[i] as number) * Math.sin((this.freq[i] as number) * t + (this.phase[i] as number));
    return Math.max(0, u);
  }

  /** Direction the wind blows from at time t (compass radians). */
  fromAt(t: number): number {
    let d = this.opts.fromDirection;
    for (let i = 0; i < 3; i++) d += (this.dirAmp[i] as number) * Math.sin((2 * Math.PI * t) / (300 * (i + 1)) + (this.dirPhase[i] as number));
    return d;
  }

  /** Wind velocity vector (NED, the direction the air moves toward). */
  velocity(t: number, out: { x: number; y: number; z: number }): { x: number; y: number; z: number } {
    const u = this.speedAt(t);
    const to = this.fromAt(t) + Math.PI;
    out.x = u * Math.cos(to);
    out.y = u * Math.sin(to);
    out.z = 0;
    return out;
  }
}
