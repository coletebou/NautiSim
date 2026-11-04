import { clamp, type Q4, qConj, qMul, qRotate, type V3, wrapPi } from './vec';

/**
 * Screen rotation relative to the device's natural portrait orientation, in degrees, as reported
 * by expo-sensors DeviceMotion.orientation: 0, 90, 180 or −90.
 */
export type ScreenRotation = 0 | 90 | 180 | -90;

/**
 * Re-express a device-frame vector (x right, y up, z out of the screen, in portrait) in screen
 * coordinates for the current UI rotation. `θ` is the angle the UI is rotated relative to the device.
 */
export function deviceToScreen(v: V3, rotation: ScreenRotation): V3 {
  const t = (rotation * Math.PI) / 180;
  const c = Math.round(Math.cos(t));
  const s = Math.round(Math.sin(t));
  return { x: c * v.x + s * v.y, y: -s * v.x + c * v.y, z: v.z };
}

/**
 * The two tilt angles used for the helm, from a gravity (or accelerometer) vector in screen
 * coordinates:
 * - `roll`: rotation about the axis through the screen (turning the phone like a steering wheel),
 *   the angle of the gravity vector projected onto the screen plane;
 * - `pitch`: rotation about the screen's horizontal axis (top edge toward or away from you), measured
 *   after removing the roll so the two do not cross-couple.
 * Both are invariant to the sign convention of the gravity vector, which differs between iOS and
 * Android, because only differences from the neutral pose are used.
 */
export interface TiltAngles {
  roll: number;
  pitch: number;
  /** How much of gravity lies in the screen plane (0 = phone flat, 1 = phone upright). */
  planarity: number;
}

export function tiltFromGravity(g: V3): TiltAngles {
  const n = Math.hypot(g.x, g.y, g.z) || 1;
  const roll = Math.atan2(g.x, -g.y);
  // undo the roll: rotate (x, y) by −roll so x vanishes
  const c = Math.cos(roll);
  const s = Math.sin(roll);
  const yp = s * g.x - c * g.y; // = |g_xy| ≥ 0 once the roll is removed
  const pitch = Math.atan2(g.z, yp);
  return { roll, pitch, planarity: Math.hypot(g.x, g.y) / n };
}

export interface NeutralPose {
  roll: number;
  pitch: number;
  rotation: ScreenRotation;
  /**
   * +1 when the sensor vector points toward the ground (iOS-style gravity), −1 when it points up
   * (accelerometer reaction). Detected at capture: nobody holds the phone screen-down.
   */
  gravitySign: 1 | -1;
  /** Attitude quaternion at capture, if the attitude source is used. */
  attitude?: Q4;
}

/** Average a short window of gravity samples into a neutral pose. */
export function captureNeutral(samples: readonly V3[], rotation: ScreenRotation): NeutralPose {
  if (samples.length === 0) return { roll: 0, pitch: 0, rotation, gravitySign: 1 };
  const m = samples.reduce((a, v) => ({ x: a.x + v.x, y: a.y + v.y, z: a.z + v.z }), { x: 0, y: 0, z: 0 });
  const k = 1 / samples.length;
  const sv = deviceToScreen({ x: m.x * k, y: m.y * k, z: m.z * k }, rotation);
  const gravitySign: 1 | -1 = -sv.y - sv.z >= 0 ? 1 : -1;
  const t = tiltFromGravity({ x: sv.x * gravitySign, y: sv.y * gravitySign, z: sv.z * gravitySign });
  return { roll: t.roll, pitch: t.pitch, rotation, gravitySign };
}

/** Tilt relative to the neutral pose (radians), wrapped to (−π, π]. */
export function relativeTilt(g: V3, neutral: NeutralPose, rotation: ScreenRotation): { roll: number; pitch: number; planarity: number } {
  const k = neutral.gravitySign;
  const t = tiltFromGravity(deviceToScreen({ x: g.x * k, y: g.y * k, z: g.z * k }, rotation));
  return { roll: wrapPi(t.roll - neutral.roll), pitch: wrapPi(t.pitch - neutral.pitch), planarity: t.planarity };
}

/**
 * Quaternion from W3C DeviceOrientation-style Euler angles (radians): alpha about z, beta about x,
 * gamma about y, applied Z-X′-Y″. expo-sensors reports DeviceMotion.rotation in this form.
 */
export function quatFromDeviceRotation(alpha: number, beta: number, gamma: number): Q4 {
  const cz = Math.cos(alpha / 2);
  const sz = Math.sin(alpha / 2);
  const cx = Math.cos(beta / 2);
  const sx = Math.sin(beta / 2);
  const cy = Math.cos(gamma / 2);
  const sy = Math.sin(gamma / 2);
  const qz = { w: cz, x: 0, y: 0, z: sz };
  const qx = { w: cx, x: sx, y: 0, z: 0 };
  const qy = { w: cy, x: 0, y: sy, z: 0 };
  return qMul(qMul(qz, qx), qy);
}

/**
 * Tilt relative to a neutral attitude from fused orientation quaternions (device → earth). The
 * earth's "down" is re-expressed in the device frame for both, then treated like gravity.
 */
export function relativeTiltFromAttitude(q: Q4, neutral: Q4, rotation: ScreenRotation): { roll: number; pitch: number } {
  const down = { x: 0, y: 0, z: -1 };
  const gNow = qRotate(qConj(q), down);
  const gRef = qRotate(qConj(neutral), down);
  const a = tiltFromGravity(deviceToScreen(gNow, rotation));
  const b = tiltFromGravity(deviceToScreen(gRef, rotation));
  return { roll: wrapPi(a.roll - b.roll), pitch: wrapPi(a.pitch - b.pitch) };
}

export interface AxisCurve {
  /** Angle below which the output is zero (rad). */
  deadzone: number;
  /** Angle at which the output reaches ±1 (rad). */
  saturation: number;
  /** 0 = linear, 1 = fully cubic (fine control near centre). */
  expo: number;
  invert?: boolean;
}

/**
 * Map an angle to −1…1 with a dead zone that rescales (so the output starts from 0 at the dead-zone
 * edge instead of jumping), a saturation angle and an expo curve.
 */
export function shapeAxis(angle: number, c: AxisCurve): number {
  const a = Math.abs(angle);
  if (a <= c.deadzone) return 0;
  const span = Math.max(c.saturation - c.deadzone, 1e-6);
  const t = clamp((a - c.deadzone) / span, 0, 1);
  const e = clamp(c.expo, 0, 1);
  const shaped = (1 - e) * t + e * t * t * t;
  const out = Math.sign(angle) * shaped;
  return c.invert ? -out : out;
}

/** Move toward a target at a limited rate (units per second). */
export function rateLimit(current: number, target: number, maxRate: number, dt: number): number {
  const d = target - current;
  const m = maxRate * dt;
  return Math.abs(d) <= m ? target : current + Math.sign(d) * m;
}
