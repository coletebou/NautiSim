import { InputMixer } from '@nautisim/controls';
import { activateKeepAwakeAsync, deactivateKeepAwake } from 'expo-keep-awake';
import { useRouter } from 'expo-router';
import * as ScreenOrientation from 'expo-screen-orientation';
import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState, useSyncExternalStore } from 'react';
import { PanResponder, Pressable, StyleSheet, Text, useWindowDimensions, View } from 'react-native';
import { Chart } from '../hud/Chart';
import { Hud } from '../hud/Hud';
import { HapticCues } from '../input/haptics';
import { useMotionHelm } from '../input/useMotionHelm';
import { webGpuAvailable } from '../render/gpu';
import { NauticalScene } from '../render/scene';
import { SimHost } from '../sim/SimHost';
import { type CameraMode, updateSettings, useSettings } from '../state/settings';
import { HelmWheel, ThrottleLever } from '../ui/controls';
import { colors } from '../ui/theme';

const GpuView = lazy(() => import('../render/GpuView'));
const CAMERAS: CameraMode[] = ['chase', 'bridge', 'orbit'];

export default function SimScreen() {
  const settings = useSettings();
  const router = useRouter();
  const { width, height } = useWindowDimensions();
  const host = useMemo(
    () => new SimHost({ hullId: settings.hullId, seaState: settings.seaState, waveFromDeg: settings.waveFromDeg }),
    [settings.hullId, settings.seaState, settings.waveFromDeg],
  );
  const gpu = useMemo(webGpuAvailable, []);
  const [gpuError, setGpuError] = useState<string | null>(null);
  const use3d = gpu && gpuError === null;
  const scene = useMemo(
    () =>
      use3d
        ? new NauticalScene({
            vessel: host.vessel,
            displayVessel: host.hull.build({ resolution: 'display' }),
            waves: host.env.waves,
            quality: settings.quality,
          })
        : null,
    [host, use3d, settings.quality],
  );
  useEffect(() => () => scene?.dispose(), [scene]);

  const motionWanted = settings.controlMode === 'motion';
  const motion = useMotionHelm(settings, motionWanted);
  const mixer = useMemo(() => new InputMixer('motion'), []);
  const touch = useRef({ helm: 0, throttle: 0 });
  const [throttle, setThrottle] = useState(0);
  const [helmSource, setHelmSource] = useState('touch');
  const haptics = useMemo(() => new HapticCues(), []);
  haptics.enabled = settings.haptics;
  const cameraRef = useRef<CameraMode>(settings.camera);
  cameraRef.current = settings.camera;
  const subscribe = useCallback((l: () => void) => host.subscribe(l), [host]);
  const snap = useSyncExternalStore(subscribe, host.getSnapshot, host.getSnapshot);

  useEffect(() => {
    ScreenOrientation.lockAsync(ScreenOrientation.OrientationLock.LANDSCAPE).catch(() => undefined);
    activateKeepAwakeAsync('sim').catch(() => undefined);
    return () => {
      ScreenOrientation.unlockAsync().catch(() => undefined);
      deactivateKeepAwake('sim');
    };
  }, []);

  const motionOk = motionWanted && motion.state.current.available;
  const onFrame = useCallback(
    (dt: number) => {
      const now = Date.now() / 1000;
      const m = motion.state.current;
      if (motionWanted && m.available) mixer.offer('motion', 'helm', m.helm, now);
      mixer.offer('touch', 'helm', touch.current.helm, now);
      const helm = mixer.read('helm', now);
      const thr = motionWanted && m.available && m.throttleActive ? m.throttle : touch.current.throttle;
      host.setControls(helm.value, thr);
      const alpha = host.advance(dt);
      if (scene) scene.frame(host.pose(alpha), host.latest, cameraRef.current);
      haptics.update(host.latest, host.vessel.actuators.rudderMax, now);
      if (helm.source && helm.source !== helmSourceRef.current) {
        helmSourceRef.current = helm.source;
        setHelmSource(helm.source);
      }
    },
    [host, scene, motion, mixer, haptics, motionWanted],
  );
  const helmSourceRef = useRef('touch');

  // without the 3D view, drive the simulation from requestAnimationFrame
  useEffect(() => {
    if (use3d) return;
    let id = 0;
    let last = 0;
    const loop = (t: number): void => {
      const dt = last === 0 ? 1 / 60 : Math.min((t - last) / 1000, 0.1);
      last = t;
      onFrame(dt);
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, [use3d, onFrame]);

  // drag anywhere on the open water to orbit the camera
  const orbitPan = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => cameraRef.current === 'orbit',
        onPanResponderMove: (_e, g) => {
          if (!scene) return;
          scene.orbit.yaw -= g.vx * 0.05;
          scene.orbit.pitch = Math.min(1.3, Math.max(0.05, scene.orbit.pitch + g.vy * 0.03));
        },
      }),
    [scene],
  );

  const cycleCamera = (): void => {
    const i = CAMERAS.indexOf(settings.camera);
    updateSettings({ camera: CAMERAS[(i + 1) % CAMERAS.length] as CameraMode });
  };
  const t = snap.t;
  const windKn = host.env.windField.speedAt(t) / (1852 / 3600);
  const windFrom = ((((host.env.windField.fromAt(t) * 180) / Math.PI) % 360) + 360) % 360;
  const showWheel = settings.controlMode === 'touch' || !motionOk;

  return (
    <View style={styles.root} {...orbitPan.panHandlers}>
      {use3d && scene ? (
        <Suspense fallback={<Text style={styles.loading}>Loading 3D…</Text>}>
          <GpuView scene={scene} onFrame={onFrame} onError={(e) => setGpuError(String(e))} />
        </Suspense>
      ) : (
        <View style={styles.chartFull}>
          <Chart snap={snap} length={host.vessel.dimensions.length} size={Math.min(width, height) * 0.9} fullscreen />
        </View>
      )}
      <Hud snap={snap} rudderMax={host.vessel.actuators.rudderMax} helmSource={helmSource} windFromDeg={windFrom} windKn={windKn} />
      {use3d && settings.showChart ? (
        <View style={styles.minimap} pointerEvents="none">
          <Chart snap={snap} length={host.vessel.dimensions.length} size={120} />
        </View>
      ) : null}
      {!use3d ? (
        <Text style={styles.banner}>
          {gpu ? `3D view failed: ${gpuError}` : '2-D chart mode: the 3D view needs a development build (WebGPU is not in Expo Go).'}
        </Text>
      ) : null}
      {motionWanted && !motionOk ? <Text style={[styles.banner, { top: 70 }]}>Motion sensors not available: using the on-screen wheel.</Text> : null}
      <View style={styles.buttons}>
        <Btn label={settings.camera.toUpperCase()} onPress={cycleCamera} />
        {motionOk ? <Btn label="CENTRE" onPress={motion.calibrate} /> : null}
        <Btn label={host.paused ? 'RESUME' : 'PAUSE'} onPress={() => (host.paused = !host.paused)} />
        <Btn label="RESET" onPress={() => host.reset()} />
        <Btn label="EXIT" onPress={() => router.back()} />
      </View>
      {settings.throttleMode === 'slider' || !motionOk ? (
        <View style={styles.lever}>
          <ThrottleLever
            value={throttle}
            allowAstern={settings.allowAstern}
            onDetent={() => haptics.detent()}
            onChange={(v) => {
              touch.current.throttle = v;
              setThrottle(v);
            }}
          />
        </View>
      ) : null}
      {showWheel ? (
        <View style={styles.wheel}>
          <HelmWheel
            size={150}
            value={0}
            onChange={(h) => {
              touch.current.helm = h;
            }}
          />
        </View>
      ) : null}
    </View>
  );
}

function Btn(props: { label: string; onPress: () => void }) {
  return (
    <Pressable onPress={props.onPress} style={styles.btn} accessibilityRole="button" accessibilityLabel={props.label}>
      <Text style={styles.btnText}>{props.label}</Text>
    </Pressable>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, backgroundColor: colors.bg },
  chartFull: { position: 'absolute', left: 0, right: 0, top: 0, bottom: 0, alignItems: 'center', justifyContent: 'center' },
  loading: { color: colors.text, alignSelf: 'center', marginTop: '20%' },
  minimap: { position: 'absolute', right: 90, bottom: 10 },
  banner: {
    position: 'absolute',
    top: 64,
    alignSelf: 'center',
    color: colors.warn,
    backgroundColor: colors.panelTranslucent,
    padding: 6,
    borderRadius: 8,
    fontSize: 12,
  },
  buttons: { position: 'absolute', top: 8, left: 8, gap: 6 },
  btn: { backgroundColor: colors.panelTranslucent, borderRadius: 8, paddingVertical: 8, paddingHorizontal: 10, minWidth: 74, alignItems: 'center' },
  btnText: { color: colors.text, fontSize: 12, fontWeight: '700' },
  lever: { position: 'absolute', right: 10, bottom: 20 },
  wheel: { position: 'absolute', left: 16, bottom: 16 },
});
