import { DEFAULT_MOTION_SETTINGS, MotionHelm, type MotionSettings, quatFromDeviceRotation, type ScreenRotation } from '@nautisim/controls';
import { DeviceMotion, type DeviceMotionMeasurement } from 'expo-sensors';
import { useEffect, useMemo, useRef } from 'react';
import type { Settings } from '../state/settings';

const DEG = Math.PI / 180;

export function motionSettingsFrom(s: Settings): MotionSettings {
  return {
    ...DEFAULT_MOTION_SETTINGS,
    source: s.tiltSource,
    throttleMode: s.throttleMode,
    steering: {
      deadzone: s.steeringDeadzoneDeg * DEG,
      saturation: Math.max(s.steeringSaturationDeg, s.steeringDeadzoneDeg + 1) * DEG,
      expo: s.steeringExpo,
      invert: s.invertSteering,
    },
    throttle: { ...DEFAULT_MOTION_SETTINGS.throttle, allowAstern: s.allowAstern },
  };
}

export interface MotionState {
  available: boolean;
  helm: number;
  throttle: number;
  throttleActive: boolean;
  /** Filtered tilt relative to neutral (rad) for the calibration UI. */
  roll: number;
  pitch: number;
  planarity: number;
  rotation: ScreenRotation;
  samples: number;
}

const toRotation = (o: number): ScreenRotation => (o === 90 || o === -90 || o === 180 ? o : 0);

/**
 * Subscribe to DeviceMotion (expo-sensors) and run the samples through MotionHelm. The result lives
 * in a ref, not React state, so 60 Hz sensor events never re-render the tree; the sim loop reads it.
 */
export function useMotionHelm(settings: Settings, enabled: boolean): { state: React.RefObject<MotionState>; calibrate: () => void } {
  // biome-ignore lint/correctness/useExhaustiveDependencies: one filter chain per mount; the effect below pushes new settings into it
  const helm = useMemo(() => new MotionHelm(motionSettingsFrom(settings)), []);
  const state = useRef<MotionState>({
    available: false,
    helm: 0,
    throttle: 0,
    throttleActive: false,
    roll: 0,
    pitch: 0,
    planarity: 1,
    rotation: 0,
    samples: 0,
  });

  useEffect(() => {
    helm.settings = motionSettingsFrom(settings);
  }, [helm, settings]);

  useEffect(() => {
    if (!enabled) return;
    let sub: { remove: () => void } | undefined;
    let cancelled = false;
    (async () => {
      const ok = await DeviceMotion.isAvailableAsync().catch(() => false);
      if (!ok || cancelled) return;
      const perm = await DeviceMotion.requestPermissionsAsync().catch(() => ({ granted: false }));
      if (!perm.granted || cancelled) return;
      state.current.available = true;
      DeviceMotion.setUpdateInterval(16);
      sub = DeviceMotion.addListener((m: DeviceMotionMeasurement) => {
        const g = m.accelerationIncludingGravity;
        if (!g) return;
        const rotation = toRotation(m.orientation);
        const rr = m.rotationRate;
        const out = helm.update({
          t: Date.now() / 1000,
          gravity: { x: g.x, y: g.y, z: g.z },
          ...(rr && { gyro: { x: rr.beta * DEG, y: rr.gamma * DEG, z: rr.alpha * DEG } }),
          ...(m.rotation && { attitude: quatFromDeviceRotation(m.rotation.alpha, m.rotation.beta, m.rotation.gamma) }),
          rotation,
        });
        const s = state.current;
        s.helm = out.helm;
        s.throttle = out.throttle;
        s.throttleActive = out.throttleActive;
        s.roll = helm.tilt.roll;
        s.pitch = helm.tilt.pitch;
        s.planarity = helm.tilt.planarity;
        s.rotation = rotation;
        s.samples++;
      });
    })();
    return () => {
      cancelled = true;
      sub?.remove();
    };
  }, [enabled, helm]);

  const calibrate = (): void => {
    helm.calibrate(state.current.rotation);
  };
  return { state, calibrate };
}
