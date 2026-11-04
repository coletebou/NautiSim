import { describe, expect, it } from 'vitest';
import { Environment, presetEnvironment, SEA_STATES } from '../src/environment';
import { createRng, deriveSeed } from '../src/random';
import { cos2sSpreading, dnvGamma, jonswap, piersonMoskowitz, spectralMoment, waveNumber } from '../src/spectrum';
import { WaveField } from '../src/waves';
import { WindField } from '../src/wind';

describe('random', () => {
  it('is deterministic and roughly uniform', () => {
    const a = createRng(42);
    const b = createRng(42);
    const xs = Array.from({ length: 10000 }, () => a.next());
    expect(xs.slice(0, 5)).toEqual(Array.from({ length: 5 }, () => b.next()));
    const mean = xs.reduce((s, x) => s + x, 0) / xs.length;
    expect(mean).toBeCloseTo(0.5, 1);
    expect(Math.min(...xs)).toBeGreaterThanOrEqual(0);
    expect(Math.max(...xs)).toBeLessThan(1);
    expect(deriveSeed(1, 'waves')).not.toBe(deriveSeed(1, 'wind'));
  });

  it('gaussian has unit variance', () => {
    const r = createRng(7);
    const xs = Array.from({ length: 20000 }, () => r.gaussian());
    const mean = xs.reduce((s, x) => s + x, 0) / xs.length;
    const v = xs.reduce((s, x) => s + (x - mean) ** 2, 0) / xs.length;
    expect(mean).toBeCloseTo(0, 1);
    expect(v).toBeCloseTo(1, 1);
  });
});

describe('validation: wave spectra (DNV-RP-C205)', () => {
  it('PM and JONSWAP integrate to Hs²/16', () => {
    for (const [hs, tp] of [
      [1, 5],
      [3, 8],
      [6, 12],
    ] as const) {
      const m0pm = spectralMoment((w) => piersonMoskowitz(w, hs, tp), 0);
      expect(4 * Math.sqrt(m0pm)).toBeCloseTo(hs, 2);
      const m0j = spectralMoment((w) => jonswap(w, hs, tp, 3.3), 0);
      // the A_γ normalisation is accurate to about 1 %
      expect(Math.abs(4 * Math.sqrt(m0j) - hs) / hs).toBeLessThan(0.01);
    }
  });

  it('JONSWAP peaks at ωp and γ follows the DNV rule', () => {
    const tp = 8;
    const wp = (2 * Math.PI) / tp;
    let best = 0;
    let wBest = 0;
    for (let w = 0.3; w < 2; w += 0.001) {
      const s = jonswap(w, 2, tp, 3.3);
      if (s > best) {
        best = s;
        wBest = w;
      }
    }
    expect(wBest).toBeCloseTo(wp, 2);
    expect(dnvGamma(4, 6)).toBe(5); // Tp/√Hs = 3 ≤ 3.6
    expect(dnvGamma(1, 6)).toBe(1); // 6 ≥ 5
    expect(dnvGamma(2, 6)).toBeCloseTo(Math.exp(5.75 - 1.15 * (6 / Math.sqrt(2))), 9);
  });

  it('cos-2s spreading integrates to one', () => {
    for (const s of [1, 4, 15]) {
      let sum = 0;
      const n = 3600;
      for (let i = 0; i < n; i++) sum += cos2sSpreading(-Math.PI + ((i + 0.5) * 2 * Math.PI) / n, 0, s) * ((2 * Math.PI) / n);
      expect(sum).toBeCloseTo(1, 4);
    }
  });

  it('dispersion relation in deep and shallow water', () => {
    const w = 1.0;
    expect(waveNumber(w)).toBeCloseTo((w * w) / 9.80665, 12);
    // shallow-water limit: c = √(g h)
    const h = 0.5;
    const k = waveNumber(0.05, h);
    expect(0.05 / k).toBeCloseTo(Math.sqrt(9.80665 * h), 3);
    const kh = waveNumber(0.8, 20);
    expect(9.80665 * kh * Math.tanh(kh * 20)).toBeCloseTo(0.64, 9);
  });
});

describe('wave field', () => {
  const sea = { hs: 2, tp: 7, fromDirection: 0, spreading: 4 };

  it('is reproducible from its seed and differs between seeds', () => {
    const a = new WaveField(sea, { seed: 3 });
    const b = new WaveField(sea, { seed: 3 });
    const c = new WaveField(sea, { seed: 4 });
    expect(a.elevation(12, -5, 33)).toBe(b.elevation(12, -5, 33));
    expect(a.elevation(12, -5, 33)).not.toBe(c.elevation(12, -5, 33));
  });

  it('realised surface statistics match Hs', () => {
    const f = new WaveField(sea, { seed: 11, components: 48 });
    let sum = 0;
    let sum2 = 0;
    let n = 0;
    for (let t = 0; t < 3600; t += 0.5) {
      const e = f.elevation(0, 0, t);
      sum += e;
      sum2 += e * e;
      n++;
    }
    const mean = sum / n;
    const hs = 4 * Math.sqrt(sum2 / n - mean * mean);
    expect(Math.abs(hs - 2) / 2).toBeLessThan(0.1);
  });

  it('waves from the north travel south', () => {
    const f = new WaveField({ ...sea, spreading: 1000 }, { seed: 1, components: 16 });
    for (let i = 0; i < f.components.count; i++) expect(f.components.kx[i] as number).toBeLessThan(0);
  });

  it('orbital velocity decays with depth and has the right magnitude', () => {
    const f = new WaveField({ hs: 2, tp: 7, fromDirection: 0, spreading: 1000 }, { seed: 2, components: 1 });
    const out = { x: 0, y: 0, z: 0 };
    let maxSurf = 0;
    let maxDeep = 0;
    for (let t = 0; t < 7; t += 0.05) {
      f.orbitalVelocity(0, 0, 0, t, out);
      maxSurf = Math.max(maxSurf, Math.hypot(out.x, out.z));
      f.orbitalVelocity(0, 0, 20, t, out);
      maxDeep = Math.max(maxDeep, Math.hypot(out.x, out.z));
    }
    const a = f.components.amplitude[0] as number;
    const w = f.components.omega[0] as number;
    const k = f.components.k[0] as number;
    expect(maxSurf).toBeCloseTo(a * w, 2);
    expect(maxDeep / maxSurf).toBeCloseTo(Math.exp(-k * 20), 2);
  });

  it('calm sea is flat', () => {
    const f = new WaveField({ hs: 0, tp: 5, fromDirection: 0 }, { seed: 1 });
    expect(f.elevation(1, 2, 3)).toBe(0);
    expect(f.components.count).toBe(0);
  });
});

describe('wind and environment', () => {
  it('gusts average to the mean speed with the requested turbulence', () => {
    const w = new WindField({ speed: 10, fromDirection: Math.PI / 2, turbulence: 0.1, seed: 5 });
    let s = 0;
    let s2 = 0;
    let n = 0;
    for (let t = 0; t < 36_000; t += 1) {
      const u = w.speedAt(t);
      s += u;
      s2 += u * u;
      n++;
    }
    const mean = s / n;
    const sd = Math.sqrt(s2 / n - mean * mean);
    expect(mean).toBeCloseTo(10, 0);
    expect(sd).toBeGreaterThan(0.6);
    expect(sd).toBeLessThan(1.4);
    // wind from the east blows toward the west (negative y in NED)
    const v = w.velocity(0, { x: 0, y: 0, z: 0 });
    expect(v.y).toBeLessThan(0);
  });

  it('environment adds current to orbital velocity and models a shoal', () => {
    const env = new Environment({
      seed: 1,
      sea: { hs: 0, tp: 5, fromDirection: 0 },
      wind: { speed: 0, fromDirection: 0 },
      current: { speed: 1, toDirection: Math.PI / 2 },
      depth: { kind: 'shoal', deep: 50, shallow: 5, distance: 1000, bearing: 0 },
    });
    const v = env.waterVelocity(0, 0, 0, 0, { x: 0, y: 0, z: 0 });
    expect(v.x).toBeCloseTo(0, 9);
    expect(v.y).toBeCloseTo(1, 9);
    expect(env.depth(0, 0)).toBe(50);
    expect(env.depth(500, 0)).toBeCloseTo(27.5, 9);
    expect(env.depth(5000, 0)).toBe(5);
  });

  it('presets cover calm to very rough', () => {
    expect(Object.keys(SEA_STATES)).toContain('moderate');
    const env = presetEnvironment('moderate', { seed: 9 });
    expect(env.waves.sea.hs).toBe(1.8);
    expect(Number.isFinite(env.elevation(0, 0, 0))).toBe(true);
  });
});
