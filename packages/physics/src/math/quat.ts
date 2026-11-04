import type { Vec3 } from './vec3';

/**
 * Unit quaternion (w, x, y, z) rotating body-frame vectors into the world (NED) frame:
 * v_world = q ⊗ v_body ⊗ q*.
 */
export interface Quat {
  w: number;
  x: number;
  y: number;
  z: number;
}

/** Roll φ (about body x), pitch θ (about y), yaw ψ (about z), ZYX convention as in Fossen. */
export interface Euler {
  roll: number;
  pitch: number;
  yaw: number;
}

export const quatIdentity = (): Quat => ({ w: 1, x: 0, y: 0, z: 0 });

export function quatMultiply(a: Readonly<Quat>, b: Readonly<Quat>): Quat {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

export const quatConjugate = (q: Readonly<Quat>): Quat => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q.w, q.x, q.y, q.z);
  if (n === 0) {
    q.w = 1;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    return q;
  }
  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
  return q;
}

/** Rotate a body vector into the world frame. Writes into `out` (may alias `v`). */
export function rotate(q: Readonly<Quat>, v: Readonly<Vec3>, out: Vec3 = { x: 0, y: 0, z: 0 }): Vec3 {
  // t = 2 (q.xyz × v); v' = v + w t + q.xyz × t
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  const x = v.x + q.w * tx + (q.y * tz - q.z * ty);
  const y = v.y + q.w * ty + (q.z * tx - q.x * tz);
  const z = v.z + q.w * tz + (q.x * ty - q.y * tx);
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}

/** Rotate a world vector into the body frame. */
export function rotateInverse(q: Readonly<Quat>, v: Readonly<Vec3>, out: Vec3 = { x: 0, y: 0, z: 0 }): Vec3 {
  return rotate({ w: q.w, x: -q.x, y: -q.y, z: -q.z }, v, out);
}

export function quatFromEuler(roll: number, pitch: number, yaw: number): Quat {
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2);
  const sy = Math.sin(yaw / 2);
  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
  };
}

export function quatToEuler(q: Readonly<Quat>): Euler {
  const sinrCosp = 2 * (q.w * q.x + q.y * q.z);
  const cosrCosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const sinyCosp = 2 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return {
    roll: Math.atan2(sinrCosp, cosrCosp),
    pitch: Math.asin(Math.max(-1, Math.min(1, sinp))),
    yaw: Math.atan2(sinyCosp, cosyCosp),
  };
}

/** Quaternion from axis (unit) and angle. */
export function quatFromAxisAngle(axis: Readonly<Vec3>, angle: number): Quat {
  const h = angle / 2;
  const s = Math.sin(h);
  return { w: Math.cos(h), x: axis.x * s, y: axis.y * s, z: axis.z * s };
}

/**
 * Advance an attitude by a body-frame angular velocity over dt with the exponential map
 * (q ← q ⊗ exp(½ ω dt)). Exact for constant ω and keeps unit length.
 */
export function quatIntegrate(q: Readonly<Quat>, omegaBody: Readonly<Vec3>, dt: number): Quat {
  const wx = omegaBody.x;
  const wy = omegaBody.y;
  const wz = omegaBody.z;
  const mag = Math.hypot(wx, wy, wz);
  if (mag * dt < 1e-12) return { ...q };
  const angle = mag * dt;
  const dq = quatFromAxisAngle({ x: wx / mag, y: wy / mag, z: wz / mag }, angle);
  return quatNormalize(quatMultiply(q, dq));
}

export function quatSlerp(a: Readonly<Quat>, b: Readonly<Quat>, t: number): Quat {
  let cos = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  let bw = b.w;
  let bx = b.x;
  let by = b.y;
  let bz = b.z;
  if (cos < 0) {
    cos = -cos;
    bw = -bw;
    bx = -bx;
    by = -by;
    bz = -bz;
  }
  if (cos > 0.9995) {
    return quatNormalize({
      w: a.w + t * (bw - a.w),
      x: a.x + t * (bx - a.x),
      y: a.y + t * (by - a.y),
      z: a.z + t * (bz - a.z),
    });
  }
  const theta = Math.acos(cos);
  const sin = Math.sin(theta);
  const ka = Math.sin((1 - t) * theta) / sin;
  const kb = Math.sin(t * theta) / sin;
  return { w: ka * a.w + kb * bw, x: ka * a.x + kb * bx, y: ka * a.y + kb * by, z: ka * a.z + kb * bz };
}
