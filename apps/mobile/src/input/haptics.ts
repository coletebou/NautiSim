import type { VesselSnapshot } from '@nautisim/physics';
import * as Haptics from 'expo-haptics';

/**
 * Haptic cues derived from the simulation, rate-limited so they stay informative:
 * - rudder reaching hard-over: light tick;
 * - slamming into a wave (sharp vertical deceleration): medium or heavy impact;
 * - throttle crossing the stop detent: selection tick (fired by the lever itself).
 */
export class HapticCues {
  enabled = true;
  private lastSlam = 0;
  private hardOver = false;

  update(s: VesselSnapshot, rudderMax: number, now: number): void {
    if (!this.enabled) return;
    const atLimit = Math.abs(s.actuators.rudder) > rudderMax * 0.98;
    if (atLimit && !this.hardOver) Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Light).catch(() => undefined);
    this.hardOver = atLimit;
    // NED body z: a large negative acceleration is the hull being thrown upward by a wave
    const a = -s.heaveAcceleration;
    if (a > 6 && now - this.lastSlam > 0.35) {
      this.lastSlam = now;
      Haptics.impactAsync(a > 14 ? Haptics.ImpactFeedbackStyle.Heavy : Haptics.ImpactFeedbackStyle.Medium).catch(() => undefined);
    }
  }

  detent(): void {
    if (this.enabled) Haptics.selectionAsync().catch(() => undefined);
  }
}
