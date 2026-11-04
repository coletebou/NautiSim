/**
 * Frame conversion between the physics world (NED: x north, y east, z down; body x forward,
 * y starboard, z down) and three.js (x east, y up, z south; objects face −z).
 *
 * The map M: (x, y, z)_NED → (y, −z, −x)_three is a proper rotation, so a quaternion converts by
 * rotating its vector part with M.
 */
export interface XYZ {
  x: number;
  y: number;
  z: number;
}

export function nedToThree(v: XYZ, out: XYZ = { x: 0, y: 0, z: 0 }): XYZ {
  const x = v.y;
  const y = -v.z;
  const z = -v.x;
  out.x = x;
  out.y = y;
  out.z = z;
  return out;
}

export function quatNedToThree(q: { w: number; x: number; y: number; z: number }): { w: number; x: number; y: number; z: number } {
  return { w: q.w, x: q.y, y: -q.z, z: -q.x };
}

/** Convert a body-frame mesh (Float64Array xyz) to three.js object-space positions. */
export function bodyPositionsToThree(p: Float64Array): Float32Array {
  const out = new Float32Array(p.length);
  for (let i = 0; i < p.length; i += 3) {
    out[i] = p[i + 1] as number;
    out[i + 1] = -(p[i + 2] as number);
    out[i + 2] = -(p[i] as number);
  }
  return out;
}
