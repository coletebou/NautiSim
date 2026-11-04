import { createRng } from './random';
import { cos2sSpreading, dnvGamma, G, jonswap, waveNumber } from './spectrum';

export interface SeaState {
  /** Significant wave height (m). 0 = flat calm. */
  hs: number;
  /** Peak period (s). */
  tp: number;
  /** Direction the waves come FROM, compass radians (0 = from north, π/2 = from east). */
  fromDirection: number;
  /** JONSWAP peak enhancement γ; defaults to the DNV recommendation for Hs, Tp. */
  gamma?: number;
  /** cos-2s spreading exponent (large = long-crested swell, ~2–5 = short-crested wind sea). */
  spreading?: number;
}

export interface WaveFieldOptions {
  seed: number;
  /** Number of regular components the sea is built from. */
  components?: number;
  /** Water depth (m), Infinity for deep water. */
  depth?: number;
  g?: number;
}

/** Plain arrays of the regular components (for renderers and serialisation). */
export interface WaveComponents {
  count: number;
  amplitude: Float64Array;
  /** Wave number vector, NED (rad/m). */
  kx: Float64Array;
  ky: Float64Array;
  k: Float64Array;
  omega: Float64Array;
  phase: Float64Array;
}

/**
 * Irregular, short-crested sea as a sum of linear (Airy) components drawn from a JONSWAP spectrum
 * with cos-2s spreading. One direction per frequency (drawn from the spreading function), random
 * frequency within each bin and random phase, all from a seeded PRNG: the same seed always
 * produces the same sea. η(x, y, t) = Σ aᵢ cos(kᵢ·x − ωᵢ t + φᵢ).
 */
export class WaveField {
  readonly sea: Readonly<SeaState>;
  readonly components: WaveComponents;
  readonly depth: number;
  private readonly g: number;

  constructor(sea: SeaState, opts: WaveFieldOptions) {
    this.sea = { ...sea };
    this.depth = opts.depth ?? Number.POSITIVE_INFINITY;
    this.g = opts.g ?? G;
    const n = sea.hs > 0 ? Math.max(1, opts.components ?? 32) : 0;
    const comp: WaveComponents = {
      count: n,
      amplitude: new Float64Array(n),
      kx: new Float64Array(n),
      ky: new Float64Array(n),
      k: new Float64Array(n),
      omega: new Float64Array(n),
      phase: new Float64Array(n),
    };
    this.components = comp;
    if (n === 0) return;
    const rng = createRng(opts.seed);
    const gamma = sea.gamma ?? dnvGamma(sea.hs, sea.tp);
    const s = sea.spreading ?? 4;
    const wp = (2 * Math.PI) / sea.tp;
    const wMin = 0.6 * wp;
    const wMax = 3.2 * wp;
    const dw = (wMax - wMin) / n;
    const propagation = sea.fromDirection + Math.PI;
    // tabulate the spreading CDF once for inverse-transform sampling
    const nTheta = 181;
    const cdf = new Float64Array(nTheta);
    let acc = 0;
    for (let j = 0; j < nTheta; j++) {
      const th = -Math.PI + (2 * Math.PI * j) / (nTheta - 1);
      acc += cos2sSpreading(th, 0, s);
      cdf[j] = acc;
    }
    for (let j = 0; j < nTheta; j++) cdf[j] = (cdf[j] as number) / acc;
    const sampleTheta = (u: number): number => {
      let lo = 0;
      let hi = nTheta - 1;
      while (hi - lo > 1) {
        const mid = (lo + hi) >> 1;
        if ((cdf[mid] as number) < u) lo = mid;
        else hi = mid;
      }
      const c0 = cdf[lo] as number;
      const c1 = cdf[hi] as number;
      const f = c1 > c0 ? (u - c0) / (c1 - c0) : 0;
      return -Math.PI + ((2 * Math.PI) / (nTheta - 1)) * (lo + f);
    };
    for (let i = 0; i < n; i++) {
      const w = wMin + (i + rng.range(0.1, 0.9)) * dw;
      const a = Math.sqrt(2 * jonswap(w, sea.hs, sea.tp, gamma) * dw);
      const theta = propagation + (s >= 200 ? 0 : sampleTheta(rng.next()));
      const k = waveNumber(w, this.depth, this.g);
      comp.amplitude[i] = a;
      comp.omega[i] = w;
      comp.k[i] = k;
      comp.kx[i] = k * Math.cos(theta);
      comp.ky[i] = k * Math.sin(theta);
      comp.phase[i] = rng.range(0, 2 * Math.PI);
    }
    // rescale so the realised variance matches Hs exactly (m0 = Hs² / 16)
    let m0 = 0;
    for (let i = 0; i < n; i++) m0 += 0.5 * (comp.amplitude[i] as number) ** 2;
    const target = (sea.hs * sea.hs) / 16;
    const f = Math.sqrt(target / m0);
    for (let i = 0; i < n; i++) comp.amplitude[i] = (comp.amplitude[i] as number) * f;
  }

  /** Free-surface elevation η (m, positive up). */
  elevation(x: number, y: number, t: number): number {
    const c = this.components;
    let eta = 0;
    for (let i = 0; i < c.count; i++) {
      eta += (c.amplitude[i] as number) * Math.cos((c.kx[i] as number) * x + (c.ky[i] as number) * y - (c.omega[i] as number) * t + (c.phase[i] as number));
    }
    return eta;
  }

  /**
   * Orbital velocity (NED, m/s) at depth `zDown` below the mean surface (0 at the surface). Linear
   * theory with the finite-depth profile; above the mean surface the surface value is used.
   */
  orbitalVelocity(x: number, y: number, zDown: number, t: number, out: { x: number; y: number; z: number }): { x: number; y: number; z: number } {
    const c = this.components;
    const d = Math.max(0, zDown);
    const h = this.depth;
    let vx = 0;
    let vy = 0;
    let vz = 0;
    for (let i = 0; i < c.count; i++) {
      const k = c.k[i] as number;
      const a = c.amplitude[i] as number;
      const w = c.omega[i] as number;
      const th = (c.kx[i] as number) * x + (c.ky[i] as number) * y - w * t + (c.phase[i] as number);
      let fh: number;
      let fv: number;
      if (!Number.isFinite(h) || k * h > 10) {
        fh = Math.exp(-k * d);
        fv = fh;
      } else {
        const sh = Math.sinh(k * h);
        fh = Math.cosh(k * (h - d)) / sh;
        fv = Math.sinh(k * (h - d)) / sh;
      }
      const uh = a * w * fh * Math.cos(th);
      vx += (uh * (c.kx[i] as number)) / k;
      vy += (uh * (c.ky[i] as number)) / k;
      // vertical velocity is up = a ω sin θ; NED z is down
      vz -= a * w * fv * Math.sin(th);
    }
    out.x = vx;
    out.y = vy;
    out.z = vz;
    return out;
  }

  /** Horizontal surface slope (∂η/∂x, ∂η/∂y) for normals and visual effects. */
  slope(x: number, y: number, t: number): { dx: number; dy: number } {
    const c = this.components;
    let dx = 0;
    let dy = 0;
    for (let i = 0; i < c.count; i++) {
      const s = -(c.amplitude[i] as number) * Math.sin((c.kx[i] as number) * x + (c.ky[i] as number) * y - (c.omega[i] as number) * t + (c.phase[i] as number));
      dx += s * (c.kx[i] as number);
      dy += s * (c.ky[i] as number);
    }
    return { dx, dy };
  }

  /** Copy of the components as Float32 arrays for a GPU or CPU renderer (largest first). */
  renderComponents(max = this.components.count): { count: number; data: Float32Array } {
    const c = this.components;
    const order = Array.from({ length: c.count }, (_, i) => i).sort((a, b) => (c.amplitude[b] as number) - (c.amplitude[a] as number));
    const n = Math.min(max, c.count);
    const data = new Float32Array(n * 5);
    for (let j = 0; j < n; j++) {
      const i = order[j] as number;
      data.set([c.amplitude[i] as number, c.kx[i] as number, c.ky[i] as number, c.omega[i] as number, c.phase[i] as number], j * 5);
    }
    return { count: n, data };
  }
}
