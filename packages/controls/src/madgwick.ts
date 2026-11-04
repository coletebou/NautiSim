import type { Q4, V3 } from './vec';

/**
 * Madgwick's IMU (gyro + accelerometer) orientation filter (Madgwick 2010), gradient-descent form.
 * Quaternion rotates the sensor frame into the earth frame with earth z up. `beta` trades gyro drift
 * correction against accelerometer noise (≈ 0.03–0.1 for phones).
 */
export class MadgwickIMU {
  q: Q4 = { w: 1, x: 0, y: 0, z: 0 };
  constructor(public beta = 0.06) {}

  reset(q: Q4 = { w: 1, x: 0, y: 0, z: 0 }): void {
    this.q = { ...q };
  }

  /** gyro in rad/s (sensor frame), accel in any unit (only the direction is used), dt in s. */
  update(gyro: V3, accel: V3, dt: number): Q4 {
    let { w: q0, x: q1, y: q2, z: q3 } = this.q;
    const gx = gyro.x;
    const gy = gyro.y;
    const gz = gyro.z;
    // rate of change from the gyroscope
    let qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    let qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    let qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    let qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    const an = Math.hypot(accel.x, accel.y, accel.z);
    if (an > 1e-9) {
      const ax = accel.x / an;
      const ay = accel.y / an;
      const az = accel.z / an;
      const _2q0 = 2 * q0;
      const _2q1 = 2 * q1;
      const _2q2 = 2 * q2;
      const _2q3 = 2 * q3;
      const _4q0 = 4 * q0;
      const _4q1 = 4 * q1;
      const _4q2 = 4 * q2;
      const _8q1 = 8 * q1;
      const _8q2 = 8 * q2;
      const q0q0 = q0 * q0;
      const q1q1 = q1 * q1;
      const q2q2 = q2 * q2;
      const q3q3 = q3 * q3;
      let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      let s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      let s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      let s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
      const sn = Math.hypot(s0, s1, s2, s3);
      if (sn > 1e-12) {
        s0 /= sn;
        s1 /= sn;
        s2 /= sn;
        s3 /= sn;
        qDot1 -= this.beta * s0;
        qDot2 -= this.beta * s1;
        qDot3 -= this.beta * s2;
        qDot4 -= this.beta * s3;
      }
    }
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    const n = Math.hypot(q0, q1, q2, q3) || 1;
    this.q = { w: q0 / n, x: q1 / n, y: q2 / n, z: q3 / n };
    return this.q;
  }
}
