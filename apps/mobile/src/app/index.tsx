import { SEA_STATES, type SeaStateId } from '@nautisim/environment';
import { HULL_CATALOG } from '@nautisim/physics';
import { useRouter } from 'expo-router';
import { Pressable, ScrollView, StyleSheet, Text, View } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { updateSettings, useSettings } from '../state/settings';
import { colors } from '../ui/theme';

/** Home: pick a hull and a sea state, then go to sea. */
export default function Home() {
  const s = useSettings();
  const router = useRouter();
  return (
    <SafeAreaView style={styles.root} edges={['bottom', 'left', 'right']}>
      <ScrollView contentContainerStyle={styles.content}>
        <Text style={styles.h1}>Choose your vessel</Text>
        {HULL_CATALOG.map((h) => {
          const selected = h.id === s.hullId;
          return (
            <Pressable
              key={h.id}
              onPress={() => updateSettings({ hullId: h.id })}
              style={[styles.card, selected && styles.cardSelected]}
              accessibilityRole="radio"
              accessibilityState={{ selected }}
            >
              <Text style={styles.cardTitle}>{h.name}</Text>
              <Text style={styles.cardBody}>{h.description}</Text>
              <Text style={styles.cardMeta}>{h.model}</Text>
            </Pressable>
          );
        })}
        <Text style={styles.h1}>Sea state</Text>
        <View style={styles.row}>
          {(Object.keys(SEA_STATES) as SeaStateId[]).map((id) => {
            const selected = id === s.seaState;
            return (
              <Pressable
                key={id}
                onPress={() => updateSettings({ seaState: id })}
                style={[styles.chip, selected && styles.chipSelected]}
                accessibilityRole="radio"
                accessibilityState={{ selected }}
              >
                <Text style={[styles.chipText, selected && styles.chipTextSelected]}>{SEA_STATES[id].label}</Text>
                <Text style={styles.chipSub}>
                  Hs {SEA_STATES[id].hs} m · wind {Math.round(SEA_STATES[id].wind * 1.944)} kn
                </Text>
              </Pressable>
            );
          })}
        </View>
        <Pressable style={styles.go} onPress={() => router.push('/sim')} accessibilityRole="button">
          <Text style={styles.goText}>Cast off</Text>
        </Pressable>
        <View style={styles.links}>
          <Pressable onPress={() => router.push('/settings')} accessibilityRole="link">
            <Text style={styles.link}>Helm & settings</Text>
          </Pressable>
          <Pressable onPress={() => router.push('/about')} accessibilityRole="link">
            <Text style={styles.link}>About</Text>
          </Pressable>
        </View>
        <Text style={styles.hint}>
          Steering: hold the phone in landscape like a steering wheel and turn it. Open Helm & settings to calibrate, change sensitivity or switch to the
          on-screen wheel.
        </Text>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, backgroundColor: colors.bg },
  content: { padding: 16, paddingBottom: 40 },
  h1: { color: colors.text, fontSize: 20, fontWeight: '700', marginTop: 12, marginBottom: 8 },
  card: { backgroundColor: colors.panel, borderRadius: 12, padding: 14, marginBottom: 10, borderWidth: 2, borderColor: 'transparent' },
  cardSelected: { borderColor: colors.accent },
  cardTitle: { color: colors.text, fontSize: 17, fontWeight: '700' },
  cardBody: { color: colors.text, fontSize: 14, marginTop: 4, opacity: 0.9 },
  cardMeta: { color: colors.dim, fontSize: 12, marginTop: 6 },
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  chip: { backgroundColor: colors.panel, borderRadius: 10, paddingVertical: 8, paddingHorizontal: 12, borderWidth: 2, borderColor: 'transparent' },
  chipSelected: { borderColor: colors.accent },
  chipText: { color: colors.text, fontSize: 14, fontWeight: '600' },
  chipTextSelected: { color: colors.accent },
  chipSub: { color: colors.dim, fontSize: 11 },
  go: { marginTop: 20, backgroundColor: colors.accent, borderRadius: 14, paddingVertical: 16, alignItems: 'center' },
  goText: { color: colors.bg, fontSize: 18, fontWeight: '800' },
  links: { flexDirection: 'row', justifyContent: 'space-around', marginTop: 16 },
  link: { color: colors.accent, fontSize: 16, padding: 8 },
  hint: { color: colors.dim, fontSize: 13, marginTop: 16, lineHeight: 19 },
});
