/**
 * Wave spectra and directional spreading (DNV-RP-C205 conventions). ω in rad/s, S(ω) in m²·s.
 */
export const G = 9.80665;

/** Pierson–Moskowitz spectrum for significant height Hs and peak period Tp. */
export function piersonMoskowitz(omega: number, hs: number, tp: number): number {
  if (omega <= 0) return 0;
  const wp = (2 * Math.PI) / tp;
  return (5 / 16) * hs * hs * wp ** 4 * omega ** -5 * Math.exp(-1.25 * (omega / wp) ** -4);
}

/** DNV-RP-C205 recommended peak enhancement factor γ for a given Hs, Tp. */
export function dnvGamma(hs: number, tp: number): number {
  const r = tp / Math.sqrt(hs);
  if (r <= 3.6) return 5;
  if (r >= 5) return 1;
  return Math.exp(5.75 - 1.15 * r);
}

/** JONSWAP spectrum: PM × normalising factor A_γ × γ^r. */
export function jonswap(omega: number, hs: number, tp: number, gamma = dnvGamma(hs, tp)): number {
  if (omega <= 0) return 0;
  const wp = (2 * Math.PI) / tp;
  const sigma = omega <= wp ? 0.07 : 0.09;
  const r = Math.exp(-((omega - wp) ** 2) / (2 * sigma * sigma * wp * wp));
  const Ag = 1 - 0.287 * Math.log(gamma);
  return Ag * piersonMoskowitz(omega, hs, tp) * gamma ** r;
}

/** Spectral moment m_n = ∫ ωⁿ S(ω) dω by midpoint quadrature. */
export function spectralMoment(S: (w: number) => number, n: number, wMin = 0.05, wMax = 8, steps = 4000): number {
  const dw = (wMax - wMin) / steps;
  let m = 0;
  for (let i = 0; i < steps; i++) {
    const w = wMin + (i + 0.5) * dw;
    m += w ** n * S(w) * dw;
  }
  return m;
}

function gammaFn(z: number): number {
  // Lanczos approximation (g = 7)
  const c = [
    0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059, 12.507343278686905, -0.13857109526572012,
    9.9843695780195716e-6, 1.5056327351493116e-7,
  ];
  if (z < 0.5) return Math.PI / (Math.sin(Math.PI * z) * gammaFn(1 - z));
  const x = z - 1;
  let a = c[0] as number;
  const t = x + 7.5;
  for (let i = 1; i < 9; i++) a += (c[i] as number) / (x + i);
  return Math.sqrt(2 * Math.PI) * t ** (x + 0.5) * Math.exp(-t) * a;
}

/**
 * cos-2s directional spreading D(θ) = Γ(s+1) / (2√π Γ(s+½)) · cos^{2s}((θ − θ0)/2), normalised so
 * that ∫ D dθ = 1 over (−π, π].
 */
export function cos2sSpreading(theta: number, theta0: number, s: number): number {
  const k = gammaFn(s + 1) / (2 * Math.sqrt(Math.PI) * gammaFn(s + 0.5));
  const c = Math.cos((theta - theta0) / 2);
  return k * Math.abs(c) ** (2 * s);
}

/** Wave number from the dispersion relation ω² = g k tanh(k h) (Newton iteration). */
export function waveNumber(omega: number, depth = Number.POSITIVE_INFINITY, g = G): number {
  const kDeep = (omega * omega) / g;
  if (!Number.isFinite(depth) || kDeep * depth > 10) return kDeep;
  let k = Math.max(kDeep, omega / Math.sqrt(g * depth));
  for (let i = 0; i < 50; i++) {
    const th = Math.tanh(k * depth);
    const f = g * k * th - omega * omega;
    const df = g * th + g * k * depth * (1 - th * th);
    const next = k - f / df;
    if (Math.abs(next - k) < 1e-12 * k) return next;
    k = next;
  }
  return k;
}
