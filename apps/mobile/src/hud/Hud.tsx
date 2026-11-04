import type { VesselSnapshot } from '@nautisim/physics';
import { StyleSheet, Text, View } from 'react-native';
import Svg, { Line, Polygon, Rect } from 'react-native-svg';
import { colors } from '../ui/theme';

const KN = 1852 / 3600;
const deg = (r: number): number => (r * 180) / Math.PI;

function Gauge(props: { label: string; value: string; unit?: string }) {
  return (
    <View style={styles.gauge}>
      <Text style={styles.gaugeLabel}>{props.label}</Text>
      <Text style={styles.gaugeValue}>
        {props.value}
        {props.unit ? <Text style={styles.gaugeUnit}> {props.unit}</Text> : null}
      </Text>
    </View>
  );
}

/** Rudder angle indicator: port red to the left, starboard green to the right. */
function RudderIndicator(props: { angle: number; max: number }) {
  const w = 180;
  const h = 26;
  const f = Math.max(-1, Math.min(1, props.angle / props.max));
  const x = w / 2 + (f * w) / 2;
  return (
    <View style={styles.rudder}>
      <Text style={styles.gaugeLabel}>
        RUDDER {Math.abs(deg(props.angle)).toFixed(0)}° {f < -0.02 ? 'P' : f > 0.02 ? 'S' : ''}
      </Text>
      <Svg width={w} height={h}>
        <Rect x={0} y={8} width={w / 2} height={10} fill={colors.port} opacity={0.35} />
        <Rect x={w / 2} y={8} width={w / 2} height={10} fill={colors.starboard} opacity={0.35} />
        <Line x1={w / 2} y1={2} x2={w / 2} y2={h - 2} stroke={colors.text} strokeWidth={1} />
        <Polygon points={`${x - 6},2 ${x + 6},2 ${x},${h - 2}`} fill={colors.text} />
      </Svg>
    </View>
  );
}

/** Heel and trim: an artificial-horizon style indicator. */
function Attitude(props: { roll: number; pitch: number }) {
  const s = 64;
  const roll = deg(props.roll);
  const dy = Math.max(-20, Math.min(20, deg(props.pitch) * 2));
  return (
    <View style={styles.attitude}>
      <Svg width={s} height={s}>
        <Rect x={0} y={0} width={s} height={s} rx={8} fill={colors.panel} />
        <Line
          x1={s / 2 - 26 * Math.cos(props.roll)}
          y1={s / 2 - 26 * Math.sin(props.roll) + dy}
          x2={s / 2 + 26 * Math.cos(props.roll)}
          y2={s / 2 + 26 * Math.sin(props.roll) + dy}
          stroke={colors.accent}
          strokeWidth={3}
        />
        <Line x1={s / 2 - 8} y1={s / 2} x2={s / 2 + 8} y2={s / 2} stroke={colors.text} strokeWidth={1} />
      </Svg>
      <Text style={styles.small}>
        HEEL {Math.abs(roll).toFixed(1)}° {roll > 0.2 ? 'S' : roll < -0.2 ? 'P' : ''}
      </Text>
      <Text style={styles.small}>TRIM {deg(props.pitch).toFixed(1)}°</Text>
    </View>
  );
}

export interface HudProps {
  snap: VesselSnapshot;
  rudderMax: number;
  helmSource: string;
  windFromDeg: number;
  windKn: number;
}

export function Hud(props: HudProps) {
  const s = props.snap;
  const hdg = (deg(s.heading) + 360) % 360;
  const cog = (deg(s.courseOverGround) + 360) % 360;
  return (
    <View pointerEvents="none" style={StyleSheet.absoluteFill}>
      <View style={styles.topBar}>
        <Gauge label="SOG" value={(s.speedOverGround / KN).toFixed(1)} unit="kn" />
        <Gauge label="STW" value={(s.speedThroughWater / KN).toFixed(1)} unit="kn" />
        <Gauge label="HDG" value={hdg.toFixed(0).padStart(3, '0')} unit="°" />
        <Gauge label="COG" value={cog.toFixed(0).padStart(3, '0')} unit="°" />
        <Gauge label="RPM" value={s.engineRpm.toFixed(0)} />
        <Gauge label="WIND" value={props.windKn.toFixed(0)} unit={`kn ${props.windFromDeg.toFixed(0).padStart(3, '0')}°`} />
      </View>
      <View style={styles.bottomCentre}>
        <RudderIndicator angle={s.actuators.rudder} max={props.rudderMax} />
        <Text style={styles.small}>helm: {props.helmSource}</Text>
      </View>
      <View style={styles.leftMid}>
        <Attitude roll={s.euler.roll} pitch={s.euler.pitch} />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  topBar: { flexDirection: 'row', alignSelf: 'center', marginTop: 8, backgroundColor: colors.panelTranslucent, borderRadius: 10, paddingHorizontal: 6 },
  gauge: { paddingHorizontal: 10, paddingVertical: 4, alignItems: 'center', minWidth: 62 },
  gaugeLabel: { color: colors.dim, fontSize: 10, fontWeight: '700', letterSpacing: 1 },
  gaugeValue: { color: colors.text, fontSize: 20, fontWeight: '700', fontVariant: ['tabular-nums'] },
  gaugeUnit: { color: colors.dim, fontSize: 11, fontWeight: '400' },
  rudder: { alignItems: 'center', backgroundColor: colors.panelTranslucent, borderRadius: 10, padding: 6 },
  bottomCentre: { position: 'absolute', bottom: 10, alignSelf: 'center', alignItems: 'center' },
  leftMid: { position: 'absolute', right: 90, top: 72 },
  attitude: { alignItems: 'center', backgroundColor: colors.panelTranslucent, borderRadius: 10, padding: 6 },
  small: { color: colors.text, fontSize: 11, fontVariant: ['tabular-nums'] },
});
