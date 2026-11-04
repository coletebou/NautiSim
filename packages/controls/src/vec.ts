/** Minimal vector/quaternion helpers so this package has no dependencies. */
export interface V3 {
  x: number;
  y: number;
  z: number;
}

export interface Q4 {
  w: number;
  x: number;
  y: number;
  z: number;
}

export const v3 = (x: number, y: number, z: number): V3 => ({ x, y, z });
export const len3 = (v: V3): number => Math.hypot(v.x, v.y, v.z);
export const clamp = (v: number, lo: number, hi: number): number => Math.min(Math.max(v, lo), hi);

export function wrapPi(a: number): number {
  const t = 2 * Math.PI;
  let r = a % t;
  if (r <= -Math.PI) r += t;
  else if (r > Math.PI) r -= t;
  return r;
}

export function qMul(a: Q4, b: Q4): Q4 {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

export const qConj = (q: Q4): Q4 => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });

export function qNormalize(q: Q4): Q4 {
  const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}

/** Rotate v by q (v' = q v q*). */
export function qRotate(q: Q4, v: V3): V3 {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return {
    x: v.x + q.w * tx + (q.y * tz - q.z * ty),
    y: v.y + q.w * ty + (q.z * tx - q.x * tz),
    z: v.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

export function qFromAxisAngle(ax: V3, angle: number): Q4 {
  const s = Math.sin(angle / 2);
  return { w: Math.cos(angle / 2), x: ax.x * s, y: ax.y * s, z: ax.z * s };
}
