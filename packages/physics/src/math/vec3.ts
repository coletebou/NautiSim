/** Plain mutable 3-vector. Functions with an `out` argument write into it and return it. */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export const vec3 = (x = 0, y = 0, z = 0): Vec3 => ({ x, y, z });
export const clone = (a: Readonly<Vec3>): Vec3 => ({ x: a.x, y: a.y, z: a.z });

export function set(out: Vec3, x: number, y: number, z: number): Vec3 {
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}

export function copy(out: Vec3, a: Readonly<Vec3>): Vec3 {
  out.x = a.x;
  out.y = a.y;
  out.z = a.z;
  return out;
}

export const add = (a: Readonly<Vec3>, b: Readonly<Vec3>): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
export const sub = (a: Readonly<Vec3>, b: Readonly<Vec3>): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
export const scale = (a: Readonly<Vec3>, k: number): Vec3 => ({ x: a.x * k, y: a.y * k, z: a.z * k });
export const dot = (a: Readonly<Vec3>, b: Readonly<Vec3>): number => a.x * b.x + a.y * b.y + a.z * b.z;
export const length = (a: Readonly<Vec3>): number => Math.hypot(a.x, a.y, a.z);

export function cross(a: Readonly<Vec3>, b: Readonly<Vec3>): Vec3 {
  return { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x };
}

export function normalize(a: Readonly<Vec3>): Vec3 {
  const l = length(a);
  return l > 0 ? scale(a, 1 / l) : vec3();
}

export function addInPlace(out: Vec3, a: Readonly<Vec3>): Vec3 {
  out.x += a.x;
  out.y += a.y;
  out.z += a.z;
  return out;
}

export function addScaledInPlace(out: Vec3, a: Readonly<Vec3>, k: number): Vec3 {
  out.x += a.x * k;
  out.y += a.y * k;
  out.z += a.z * k;
  return out;
}

/** out += r × f (accumulate the moment of force f applied at arm r). */
export function addCrossInPlace(out: Vec3, r: Readonly<Vec3>, f: Readonly<Vec3>): Vec3 {
  out.x += r.y * f.z - r.z * f.y;
  out.y += r.z * f.x - r.x * f.z;
  out.z += r.x * f.y - r.y * f.x;
  return out;
}

export function zero(out: Vec3): Vec3 {
  out.x = 0;
  out.y = 0;
  out.z = 0;
  return out;
}
