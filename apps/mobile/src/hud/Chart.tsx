import type { VesselSnapshot } from '@nautisim/physics';
import { useRef } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import Svg, { Circle, Line, Polygon, Polyline } from 'react-native-svg';
import { colors } from '../ui/theme';

/**
 * Top-down chart: north up, the vessel's track and range rings. A minimap over the 3D view, and the
 * main view when the 3D renderer is unavailable (Expo Go has no WebGPU module).
 */
export function Chart(props: { snap: VesselSnapshot; length: number; size: number; fullscreen?: boolean }) {
  const track = useRef<Array<{ x: number; y: number }>>([]);
  const s = props.snap;
  const last = track.current[track.current.length - 1];
  const spacing = Math.max(2, props.length / 4);
  if (!last || Math.hypot(s.position.x - last.x, s.position.y - last.y) > spacing) {
    track.current.push({ x: s.position.x, y: s.position.y });
    if (track.current.length > 600) track.current.shift();
  }
  const size = props.size;
  const range = Math.max(150, props.length * 12); // metres from centre to edge
  const k = size / 2 / range;
  // NED → screen: north up, east right, centred on the vessel
  const toScreen = (x: number, y: number): string => `${size / 2 + (y - s.position.y) * k},${size / 2 - (x - s.position.x) * k}`;
  const pts = track.current.map((p) => toScreen(p.x, p.y)).join(' ');
  const h = s.heading;
  const boat = Math.max(6, props.length * k);
  const c = Math.cos(h);
  const sn = Math.sin(h);
  const hull = (
    [
      [0, -boat],
      [boat * 0.35, boat * 0.5],
      [-boat * 0.35, boat * 0.5],
    ] as const
  )
    .map(([px, py]) => `${size / 2 + px * c - py * sn},${size / 2 + px * sn + py * c}`)
    .join(' ');
  return (
    <View style={[styles.wrap, props.fullscreen && styles.full]} pointerEvents="none">
      <Svg width={size} height={size}>
        <Circle cx={size / 2} cy={size / 2} r={size / 2 - 2} stroke={colors.accentDim} strokeWidth={1} fill="rgba(6,18,31,0.6)" />
        <Circle cx={size / 2} cy={size / 2} r={size / 4} stroke={colors.accentDim} strokeWidth={1} fill="none" />
        <Line x1={size / 2} y1={4} x2={size / 2} y2={14} stroke={colors.text} strokeWidth={2} />
        <Polyline points={pts} stroke={colors.accent} strokeWidth={1.5} fill="none" />
        <Polygon points={hull} fill={colors.text} />
      </Svg>
      <Text style={styles.scale}>inner ring {Math.round(range / 2)} m · north up</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { alignItems: 'center' },
  full: { flex: 1, justifyContent: 'center' },
  scale: { color: colors.dim, fontSize: 10, marginTop: 2 },
});
