import { useEffect, useRef } from 'react';
import { PixelRatio, StyleSheet, View } from 'react-native';
import { Canvas, type CanvasRef } from 'react-native-webgpu';
import * as THREE from 'three/webgpu';
import type { NauticalScene } from './scene';

export interface GpuViewProps {
  scene: NauticalScene;
  /** Called once per display frame with the frame time step (s); advances physics and the scene. */
  onFrame: (dt: number) => void;
  onError?: (e: unknown) => void;
}

/**
 * three.js WebGPURenderer on a react-native-webgpu canvas (Metal on iOS, Vulkan on Android, via
 * Dawn). The render loop runs on the JS thread; React state is never touched per frame.
 */
export default function GpuView({ scene, onFrame, onError }: GpuViewProps) {
  const ref = useRef<CanvasRef>(null);
  const frameRef = useRef(onFrame);
  frameRef.current = onFrame;

  useEffect(() => {
    let renderer: THREE.WebGPURenderer | undefined;
    let disposed = false;
    (async () => {
      try {
        const context = ref.current?.getContext('webgpu');
        if (!context) throw new Error('WebGPU context unavailable');
        const adapter = await navigator.gpu.requestAdapter();
        if (!adapter) throw new Error('No WebGPU adapter');
        const device = await adapter.requestDevice();
        if (disposed) return;
        const canvas = context.canvas as unknown as HTMLCanvasElement;
        const ratio = Math.min(PixelRatio.get(), 2);
        canvas.width = canvas.clientWidth * ratio;
        canvas.height = canvas.clientHeight * ratio;
        renderer = new THREE.WebGPURenderer({ antialias: true, canvas, context, device } as unknown as ConstructorParameters<typeof THREE.WebGPURenderer>[0]);
        await renderer.init();
        if (disposed) return;
        scene.resize(canvas.clientWidth, canvas.clientHeight);
        let last = 0;
        renderer.setAnimationLoop((time: number) => {
          const dt = last === 0 ? 1 / 60 : Math.min((time - last) / 1000, 0.1);
          last = time;
          frameRef.current(dt);
          renderer?.render(scene.scene, scene.camera);
          context.present();
        });
      } catch (e) {
        onError?.(e);
      }
    })();
    return () => {
      disposed = true;
      renderer?.setAnimationLoop(null);
      renderer?.dispose();
    };
  }, [scene, onError]);

  return (
    <View style={StyleSheet.absoluteFill}>
      <Canvas ref={ref} style={StyleSheet.absoluteFill} />
    </View>
  );
}
