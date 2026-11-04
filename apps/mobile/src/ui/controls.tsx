import { dragWheel, wheelToHelm } from '@nautisim/controls';
import { useRef, useState } from 'react';
import { type LayoutChangeEvent, PanResponder, StyleSheet, Text, View } from 'react-native';
import Svg, { Circle, G, Line, Rect } from 'react-native-svg';
import { colors } from './theme';

/** Horizontal slider for settings (0…1 mapped to [min, max]). */
export function Slider(props: {
  value: number;
  min: number;
  max: number;
  step?: number;
  onChange: (v: number) => void;
  label: string;
  format?: (v: number) => string;
}) {
  const width = useRef(1);
  const latest = useRef(props);
  latest.current = props;
  const { value, min, max } = props;
  const set = (x: number): void => {
    const p = latest.current;
    let v = p.min + Math.min(Math.max(x / width.current, 0), 1) * (p.max - p.min);
    if (p.step) v = Math.round(v / p.step) * p.step;
    p.onChange(v);
  };
  const pan = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onPanResponderGrant: (e) => set(e.nativeEvent.locationX),
      onPanResponderMove: (e) => set(e.nativeEvent.locationX),
    }),
  ).current;
  const f = (value - min) / (max - min);
  return (
    <View style={styles.sliderRow}>
      <Text style={styles.sliderLabel}>
        {props.label}: <Text style={styles.sliderValue}>{props.format ? props.format(value) : value.toFixed(2)}</Text>
      </Text>
      <View
        style={styles.sliderTrack}
        onLayout={(e: LayoutChangeEvent) => {
          width.current = e.nativeEvent.layout.width;
        }}
        {...pan.panHandlers}
      >
        <View style={[styles.sliderFill, { width: `${Math.round(f * 100)}%` }]} />
        <View style={[styles.sliderKnob, { left: `${Math.round(f * 100)}%` }]} />
      </View>
    </View>
  );
}

/**
 * Engine telegraph style throttle lever: drag up for ahead, down for astern, with a detent at
 * stop. Calls onChange with −1…1.
 */
export function ThrottleLever(props: { value: number; onChange: (v: number) => void; allowAstern: boolean; onDetent?: () => void }) {
  const height = useRef(1);
  const last = useRef(props.value);
  const latest = useRef(props);
  latest.current = props;
  const set = (y: number): void => {
    const p = latest.current;
    const h = height.current;
    let v = 1 - (2 * Math.min(Math.max(y, 0), h)) / h; // top = +1, bottom = −1
    if (Math.abs(v) < 0.06) v = 0;
    if (!p.allowAstern) v = Math.max(0, v);
    if (Math.sign(v) !== Math.sign(last.current) && p.onDetent) p.onDetent();
    last.current = v;
    p.onChange(v);
  };
  const pan = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onPanResponderGrant: (e) => set(e.nativeEvent.locationY),
      onPanResponderMove: (e) => set(e.nativeEvent.locationY),
    }),
  ).current;
  const v = props.value;
  return (
    <View
      style={styles.lever}
      onLayout={(e) => {
        height.current = e.nativeEvent.layout.height;
      }}
      {...pan.panHandlers}
      accessibilityRole="adjustable"
      accessibilityLabel="Throttle"
      accessibilityValue={{ min: -100, max: 100, now: Math.round(v * 100) }}
    >
      <View style={styles.leverZero} />
      <View
        style={[styles.leverFill, v >= 0 ? { bottom: '50%', height: `${v * 50}%` } : { top: '50%', height: `${-v * 50}%`, backgroundColor: colors.astern }]}
      />
      <Text style={styles.leverText}>{v === 0 ? 'STOP' : `${v > 0 ? 'AHD' : 'AST'} ${Math.round(Math.abs(v) * 100)}`}</Text>
    </View>
  );
}

/** On-screen ship's wheel: drag around it to steer. Spring-free: it stays where you leave it. */
export function HelmWheel(props: { size: number; onChange: (helm: number) => void; value: number }) {
  const lock = Math.PI; // half a turn each way for full rudder
  const state = useRef({ angle: props.value * lock });
  const lastTouch = useRef<{ x: number; y: number } | null>(null);
  const [, force] = useState(0);
  const latest = useRef(props);
  latest.current = props;
  const c = { x: props.size / 2, y: props.size / 2 };
  const pan = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onPanResponderGrant: (e) => {
        lastTouch.current = { x: e.nativeEvent.locationX, y: e.nativeEvent.locationY };
      },
      onPanResponderMove: (e) => {
        const t = { x: e.nativeEvent.locationX, y: e.nativeEvent.locationY };
        if (lastTouch.current) {
          state.current = dragWheel(state.current, c, lastTouch.current, t, lock);
          latest.current.onChange(wheelToHelm(state.current, lock));
          force((n) => n + 1);
        }
        lastTouch.current = t;
      },
      onPanResponderRelease: () => {
        lastTouch.current = null;
      },
    }),
  ).current;
  const r = props.size / 2 - 6;
  const deg = (state.current.angle * 180) / Math.PI;
  return (
    <View style={{ width: props.size, height: props.size }} {...pan.panHandlers} accessibilityLabel="Helm wheel">
      <Svg width={props.size} height={props.size}>
        <G rotation={deg} origin={`${c.x}, ${c.y}`}>
          <Circle cx={c.x} cy={c.y} r={r} stroke={colors.brass} strokeWidth={8} fill="none" />
          <Circle cx={c.x} cy={c.y} r={r * 0.22} fill={colors.brass} />
          {SPOKES.map((spoke) => {
            const a = (spoke * Math.PI) / 3;
            return (
              <Line
                key={`spoke-${spoke}`}
                x1={c.x}
                y1={c.y}
                x2={c.x + Math.sin(a) * (r + 4)}
                y2={c.y - Math.cos(a) * (r + 4)}
                stroke={colors.brass}
                strokeWidth={5}
                strokeLinecap="round"
              />
            );
          })}
          <Rect x={c.x - 3} y={c.y - r - 9} width={6} height={10} fill={colors.accent} />
        </G>
      </Svg>
    </View>
  );
}

const SPOKES = [0, 1, 2, 3, 4, 5] as const;

const styles = StyleSheet.create({
  sliderRow: { marginVertical: 8 },
  sliderLabel: { color: colors.text, fontSize: 14, marginBottom: 6 },
  sliderValue: { color: colors.accent, fontWeight: '600' },
  sliderTrack: { height: 28, borderRadius: 14, backgroundColor: colors.panel, justifyContent: 'center', overflow: 'visible' },
  sliderFill: { position: 'absolute', left: 0, top: 0, bottom: 0, borderRadius: 14, backgroundColor: colors.accentDim },
  sliderKnob: { position: 'absolute', width: 24, height: 24, marginLeft: -12, borderRadius: 12, backgroundColor: colors.accent },
  lever: {
    width: 64,
    height: 220,
    borderRadius: 12,
    backgroundColor: colors.panelTranslucent,
    overflow: 'hidden',
    alignItems: 'center',
    justifyContent: 'center',
  },
  leverZero: { position: 'absolute', left: 0, right: 0, top: '50%', height: 2, backgroundColor: colors.text },
  leverFill: { position: 'absolute', left: 8, right: 8, backgroundColor: colors.ahead, borderRadius: 6 },
  leverText: { color: colors.text, fontSize: 11, fontWeight: '700', position: 'absolute', bottom: 6 },
});
