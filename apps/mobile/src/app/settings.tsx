import { SEA_STATES } from '@nautisim/environment';
import { useEffect, useState } from 'react';
import { Pressable, ScrollView, StyleSheet, Switch, Text, View } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { useMotionHelm } from '../input/useMotionHelm';
import { DEFAULT_SETTINGS, type Settings, updateSettings, useSettings } from '../state/settings';
import { Slider } from '../ui/controls';
import { colors } from '../ui/theme';

function Choice<T extends string>(props: { label: string; value: T; options: ReadonlyArray<{ id: T; label: string }>; onChange: (v: T) => void }) {
  return (
    <View style={styles.block}>
      <Text style={styles.label}>{props.label}</Text>
      <View style={styles.row}>
        {props.options.map((o) => {
          const selected = o.id === props.value;
          return (
            <Pressable
              key={o.id}
              onPress={() => props.onChange(o.id)}
              style={[styles.chip, selected && styles.chipSelected]}
              accessibilityRole="radio"
              accessibilityState={{ selected }}
            >
              <Text style={[styles.chipText, selected && styles.chipTextSelected]}>{o.label}</Text>
            </Pressable>
          );
        })}
      </View>
    </View>
  );
}

function Toggle(props: { label: string; value: boolean; onChange: (v: boolean) => void; hint?: string }) {
  return (
    <View style={styles.toggle}>
      <View style={{ flex: 1 }}>
        <Text style={styles.label}>{props.label}</Text>
        {props.hint ? <Text style={styles.hint}>{props.hint}</Text> : null}
      </View>
      <Switch
        value={props.value}
        onValueChange={props.onChange}
        trackColor={{ true: colors.accentDim, false: colors.panel }}
        thumbColor={props.value ? colors.accent : colors.dim}
      />
    </View>
  );
}

/** Live view of the tilt reading and resulting helm, for calibrating motion steering. */
function CalibrationPanel({ settings }: { settings: Settings }) {
  const motion = useMotionHelm(settings, settings.controlMode === 'motion');
  const [, tick] = useState(0);
  useEffect(() => {
    const id = setInterval(() => tick((n) => n + 1), 100);
    return () => clearInterval(id);
  }, []);
  const m = motion.state.current;
  const deg = (r: number): string => `${((r * 180) / Math.PI).toFixed(1)}°`;
  const bar = (v: number, color: string) => (
    <View style={styles.barTrack}>
      <View style={styles.barMid} />
      <View style={[styles.barFill, { backgroundColor: color, left: v >= 0 ? '50%' : `${50 + v * 50}%`, width: `${Math.abs(v) * 50}%` }]} />
    </View>
  );
  if (settings.controlMode !== 'motion') return <Text style={styles.hint}>Motion steering is off; the on-screen wheel is used.</Text>;
  if (!m.available) return <Text style={styles.hint}>Waiting for motion sensors… (permission is needed on iOS; simulators have no sensors)</Text>;
  return (
    <View style={styles.calib}>
      <Text style={styles.label}>Live helm</Text>
      {bar(m.helm, colors.accent)}
      <Text style={styles.hint}>
        wheel tilt {deg(m.roll)} · pitch {deg(m.pitch)} · {m.planarity < 0.35 ? 'hold the phone more upright for reliable steering' : 'pose OK'}
      </Text>
      {settings.throttleMode === 'pitch' ? (
        <>
          <Text style={styles.label}>Live throttle</Text>
          {bar(m.throttle, colors.ahead)}
        </>
      ) : null}
      <Pressable style={styles.button} onPress={motion.calibrate} accessibilityRole="button">
        <Text style={styles.buttonText}>Set this pose as centre</Text>
      </Pressable>
      <Text style={styles.hint}>
        Hold the phone in landscape the way you want to sail, then tap. You can also re-centre during a run with the CENTRE button.
      </Text>
    </View>
  );
}

export default function SettingsScreen() {
  const s = useSettings();
  return (
    <SafeAreaView style={styles.root} edges={['bottom', 'left', 'right']}>
      <ScrollView contentContainerStyle={styles.content}>
        <Text style={styles.h1}>Helm</Text>
        <Choice
          label="Steering"
          value={s.controlMode}
          options={[
            { id: 'motion', label: 'Tilt the phone' },
            { id: 'touch', label: 'On-screen wheel' },
          ]}
          onChange={(v) => updateSettings({ controlMode: v })}
        />
        <Choice
          label="Throttle"
          value={s.throttleMode}
          options={[
            { id: 'slider', label: 'Lever on screen' },
            { id: 'pitch', label: 'Tilt top edge away' },
          ]}
          onChange={(v) => updateSettings({ throttleMode: v })}
        />
        <Choice
          label="Tilt sensor"
          value={s.tiltSource}
          options={[
            { id: 'gravity', label: 'Gravity (robust)' },
            { id: 'attitude', label: 'Fused attitude' },
            { id: 'fusion', label: 'Gravity + gyro' },
          ]}
          onChange={(v) => updateSettings({ tiltSource: v })}
        />
        <CalibrationPanel settings={s} />
        <Slider
          label="Tilt for full rudder"
          value={s.steeringSaturationDeg}
          min={10}
          max={70}
          step={1}
          format={(v) => `${v.toFixed(0)}°`}
          onChange={(v) => updateSettings({ steeringSaturationDeg: v })}
        />
        <Slider
          label="Dead zone"
          value={s.steeringDeadzoneDeg}
          min={0}
          max={10}
          step={0.5}
          format={(v) => `${v.toFixed(1)}°`}
          onChange={(v) => updateSettings({ steeringDeadzoneDeg: v })}
        />
        <Slider label="Centre fineness (expo)" value={s.steeringExpo} min={0} max={1} step={0.05} onChange={(v) => updateSettings({ steeringExpo: v })} />
        <Toggle
          label="Invert steering"
          value={s.invertSteering}
          onChange={(v) => updateSettings({ invertSteering: v })}
          hint="If the boat turns the wrong way when you tilt."
        />
        <Toggle label="Allow astern" value={s.allowAstern} onChange={(v) => updateSettings({ allowAstern: v })} />
        <Toggle
          label="Haptics"
          value={s.haptics}
          onChange={(v) => updateSettings({ haptics: v })}
          hint="Ticks at hard-over and the stop detent, thumps when slamming into waves."
        />

        <Text style={styles.h1}>Sea and view</Text>
        <Choice
          label="Sea state"
          value={s.seaState}
          options={(Object.keys(SEA_STATES) as Array<keyof typeof SEA_STATES>).map((id) => ({ id, label: SEA_STATES[id].label }))}
          onChange={(v) => updateSettings({ seaState: v })}
        />
        <Slider
          label="Waves and wind from"
          value={s.waveFromDeg}
          min={0}
          max={359}
          step={5}
          format={(v) => `${v.toFixed(0).padStart(3, '0')}°`}
          onChange={(v) => updateSettings({ waveFromDeg: v })}
        />
        <Choice
          label="Camera"
          value={s.camera}
          options={[
            { id: 'chase', label: 'Chase' },
            { id: 'bridge', label: 'Bridge' },
            { id: 'orbit', label: 'Orbit' },
          ]}
          onChange={(v) => updateSettings({ camera: v })}
        />
        <Choice
          label="Graphics quality"
          value={s.quality}
          options={[
            { id: 'low', label: 'Low' },
            { id: 'medium', label: 'Medium' },
            { id: 'high', label: 'High' },
          ]}
          onChange={(v) => updateSettings({ quality: v })}
        />
        <Toggle label="Mini chart" value={s.showChart} onChange={(v) => updateSettings({ showChart: v })} />
        <Pressable style={[styles.button, { backgroundColor: colors.panel }]} onPress={() => updateSettings(DEFAULT_SETTINGS)} accessibilityRole="button">
          <Text style={[styles.buttonText, { color: colors.text }]}>Reset to defaults</Text>
        </Pressable>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, backgroundColor: colors.bg },
  content: { padding: 16, paddingBottom: 48 },
  h1: { color: colors.text, fontSize: 20, fontWeight: '700', marginTop: 12, marginBottom: 4 },
  block: { marginVertical: 8 },
  label: { color: colors.text, fontSize: 14, marginBottom: 6 },
  hint: { color: colors.dim, fontSize: 12, lineHeight: 17, marginVertical: 4 },
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  chip: { backgroundColor: colors.panel, borderRadius: 10, paddingVertical: 8, paddingHorizontal: 12, borderWidth: 2, borderColor: 'transparent' },
  chipSelected: { borderColor: colors.accent },
  chipText: { color: colors.text, fontSize: 13 },
  chipTextSelected: { color: colors.accent, fontWeight: '700' },
  toggle: { flexDirection: 'row', alignItems: 'center', marginVertical: 8 },
  calib: { backgroundColor: colors.panel, borderRadius: 12, padding: 12, marginVertical: 8 },
  barTrack: { height: 16, backgroundColor: colors.bg, borderRadius: 8, overflow: 'hidden', marginBottom: 6 },
  barMid: { position: 'absolute', left: '50%', top: 0, bottom: 0, width: 2, backgroundColor: colors.dim },
  barFill: { position: 'absolute', top: 2, bottom: 2, borderRadius: 6 },
  button: { backgroundColor: colors.accent, borderRadius: 10, paddingVertical: 12, alignItems: 'center', marginTop: 10 },
  buttonText: { color: colors.bg, fontWeight: '800', fontSize: 15 },
});
