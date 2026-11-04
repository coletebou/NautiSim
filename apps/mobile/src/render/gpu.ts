import { TurboModuleRegistry } from 'react-native';

/**
 * Whether the react-native-webgpu native module is linked into this binary. It is in development
 * and store builds (expo run / EAS Build) and missing in Expo Go, where the app falls back to the
 * 2-D chart view.
 */
export function webGpuAvailable(): boolean {
  try {
    return TurboModuleRegistry.get('WebGPUModule') != null;
  } catch {
    return false;
  }
}
