/**
 * Signal filters for noisy sensor input. All are explicit-state classes with a `reset()`; time
 * steps come from sample timestamps so behaviour does not depend on the update rate.
 */

/** First-order low-pass with a cutoff frequency (Hz). */
export class LowPass {
  private y: number | undefined;
  constructor(public cutoffHz: number) {}
  static alpha(cutoffHz: number, dt: number): number {
    const tau = 1 / (2 * Math.PI * cutoffHz);
    return dt / (dt + tau);
  }
  filter(x: number, dt: number): number {
    if (this.y === undefined || !(dt > 0)) {
      this.y = x;
      return x;
    }
    this.y += LowPass.alpha(this.cutoffHz, dt) * (x - this.y);
    return this.y;
  }
  reset(): void {
    this.y = undefined;
  }
  get value(): number | undefined {
    return this.y;
  }
}

export interface OneEuroOptions {
  /** Minimum cutoff (Hz): lower = less jitter when still. */
  minCutoff: number;
  /** Speed coefficient: higher = less lag when moving fast. */
  beta: number;
  /** Cutoff for the derivative (Hz). */
  dCutoff?: number;
}

/** The 1€ filter (Casiez, Roussel & Vogel, CHI 2012): a speed-adaptive low-pass. */
export class OneEuroFilter {
  private x: LowPass;
  private dx: LowPass;
  private last: number | undefined;
  constructor(private readonly o: OneEuroOptions) {
    this.x = new LowPass(o.minCutoff);
    this.dx = new LowPass(o.dCutoff ?? 1);
  }
  filter(value: number, dt: number): number {
    if (this.last === undefined || !(dt > 0)) {
      this.last = value;
      this.x.reset();
      this.dx.reset();
      this.x.filter(value, 0);
      this.dx.filter(0, 0);
      return value;
    }
    const prev = this.x.value ?? value;
    const d = this.dx.filter((value - prev) / dt, dt);
    this.x.cutoffHz = this.o.minCutoff + this.o.beta * Math.abs(d);
    this.last = value;
    return this.x.filter(value, dt);
  }
  reset(): void {
    this.last = undefined;
    this.x.reset();
    this.dx.reset();
  }
}

/**
 * Complementary filter for one tilt angle: integrate the gyro rate (smooth, drifts) and pull toward
 * the accelerometer angle (noisy, drift-free) with time constant tau (s).
 */
export class ComplementaryFilter {
  private angle: number | undefined;
  constructor(public tau = 0.5) {}
  filter(accelAngle: number, gyroRate: number, dt: number): number {
    if (this.angle === undefined || !(dt > 0)) {
      this.angle = accelAngle;
      return accelAngle;
    }
    const a = this.tau / (this.tau + dt);
    this.angle = a * (this.angle + gyroRate * dt) + (1 - a) * accelAngle;
    return this.angle;
  }
  reset(): void {
    this.angle = undefined;
  }
}
