export const clamp = (v: number, lo: number, hi: number): number => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a: number, b: number, t: number): number => a + (b - a) * t;
export const sign = (v: number): number => (v > 0 ? 1 : v < 0 ? -1 : 0);

/** Smooth 0→1 ramp between edge0 and edge1 (cubic Hermite). */
export function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

/** Wrap an angle to (−π, π]. */
export function wrapPi(a: number): number {
  const twoPi = 2 * Math.PI;
  let r = a % twoPi;
  if (r <= -Math.PI) r += twoPi;
  else if (r > Math.PI) r -= twoPi;
  return r;
}

/** Wrap an angle to [0, 2π). */
export function wrap2Pi(a: number): number {
  const twoPi = 2 * Math.PI;
  const r = a % twoPi;
  return r < 0 ? r + twoPi : r;
}

/** Move `current` toward `target` by at most `maxDelta`. */
export function approach(current: number, target: number, maxDelta: number): number {
  const d = target - current;
  if (Math.abs(d) <= maxDelta) return target;
  return current + Math.sign(d) * maxDelta;
}

/** Bisection root finder on [lo, hi]; f(lo) and f(hi) must bracket a root. */
export function bisect(f: (x: number) => number, lo: number, hi: number, tol = 1e-10, maxIter = 200): number {
  let flo = f(lo);
  let a = lo;
  let b = hi;
  for (let i = 0; i < maxIter; i++) {
    const mid = 0.5 * (a + b);
    const fm = f(mid);
    if (Math.abs(b - a) < tol || fm === 0) return mid;
    if (Math.sign(fm) === Math.sign(flo)) {
      a = mid;
      flo = fm;
    } else {
      b = mid;
    }
  }
  return 0.5 * (a + b);
}

/** ITTC-1957 model-ship correlation line. */
export function ittc57Cf(reynolds: number): number {
  const re = Math.max(reynolds, 1e4);
  const l = Math.log10(re) - 2;
  return 0.075 / (l * l);
}

/** Schoenherr friction line (solved by Newton iteration): 0.242/√Cf = log10(Re·Cf). */
export function schoenherrCf(reynolds: number): number {
  const re = Math.max(reynolds, 1e4);
  let cf = ittc57Cf(re);
  for (let i = 0; i < 30; i++) {
    const s = Math.sqrt(cf);
    const g = 0.242 / s - Math.log10(re * cf);
    const dg = -0.121 / (cf * s) - 1 / (cf * Math.LN10);
    const next = cf - g / dg;
    if (!(next > 0)) break;
    if (Math.abs(next - cf) < 1e-12) {
      cf = next;
      break;
    }
    cf = next;
  }
  return cf;
}
