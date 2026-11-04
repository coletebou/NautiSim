import type { SeaStateId } from '@nautisim/environment';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useSyncExternalStore } from 'react';

export type ControlMode = 'motion' | 'touch';
export type TiltSourceSetting = 'gravity' | 'attitude' | 'fusion';
export type ThrottleModeSetting = 'slider' | 'pitch';
export type CameraMode = 'chase' | 'bridge' | 'orbit';
export type Quality = 'low' | 'medium' | 'high';

export interface Settings {
  hullId: string;
  seaState: SeaStateId;
  /** Waves come from this compass bearing (degrees). */
  waveFromDeg: number;
  controlMode: ControlMode;
  tiltSource: TiltSourceSetting;
  throttleMode: ThrottleModeSetting;
  /** Tilt (degrees) for full helm. */
  steeringSaturationDeg: number;
  steeringDeadzoneDeg: number;
  /** 0…1, how much finer the helm is near centre. */
  steeringExpo: number;
  invertSteering: boolean;
  allowAstern: boolean;
  haptics: boolean;
  camera: CameraMode;
  quality: Quality;
  showChart: boolean;
}

export const DEFAULT_SETTINGS: Settings = {
  hullId: 'rib',
  seaState: 'slight',
  waveFromDeg: 45,
  controlMode: 'motion',
  tiltSource: 'gravity',
  throttleMode: 'slider',
  steeringSaturationDeg: 35,
  steeringDeadzoneDeg: 2,
  steeringExpo: 0.35,
  invertSteering: false,
  allowAstern: true,
  haptics: true,
  camera: 'chase',
  quality: 'medium',
  showChart: true,
};

const KEY = 'nautisim.settings.v1';
let current: Settings = DEFAULT_SETTINGS;
const listeners = new Set<() => void>();
let loaded = false;

function emit(): void {
  for (const l of listeners) l();
}

export async function loadSettings(): Promise<Settings> {
  if (loaded) return current;
  try {
    const raw = await AsyncStorage.getItem(KEY);
    if (raw) current = { ...DEFAULT_SETTINGS, ...(JSON.parse(raw) as Partial<Settings>) };
  } catch {
    // corrupt or unavailable storage: keep the defaults
  }
  loaded = true;
  emit();
  return current;
}

export function getSettings(): Settings {
  return current;
}

export function updateSettings(patch: Partial<Settings>): void {
  current = { ...current, ...patch };
  emit();
  AsyncStorage.setItem(KEY, JSON.stringify(current)).catch(() => undefined);
}

export function subscribeSettings(l: () => void): () => void {
  listeners.add(l);
  return () => listeners.delete(l);
}

export function useSettings(): Settings {
  return useSyncExternalStore(subscribeSettings, getSettings, getSettings);
}
