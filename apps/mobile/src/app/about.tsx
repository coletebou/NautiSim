import { HULL_CATALOG } from '@nautisim/physics';
import * as Linking from 'expo-linking';
import { ScrollView, StyleSheet, Text } from 'react-native';
import { webGpuAvailable } from '../render/gpu';
import { colors } from '../ui/theme';

export default function About() {
  return (
    <ScrollView style={styles.root} contentContainerStyle={styles.content}>
      <Text style={styles.h1}>NautiSim 0.1</Text>
      <Text style={styles.p}>
        An open-source marine bridge simulator. Every hull runs on a deterministic 6-degree-of-freedom physics engine: buoyancy from the actual hull shape on
        the actual waves you see, the MMG standard method for the tanker, and Savitsky's planing equations for the small boats.
      </Text>
      <Text style={styles.h2}>Vessels</Text>
      {HULL_CATALOG.map((h) => (
        <Text key={h.id} style={styles.p}>
          • {h.name}: {h.model}
        </Text>
      ))}
      <Text style={styles.h2}>Renderer</Text>
      <Text style={styles.p}>
        {webGpuAvailable()
          ? '3D: three.js on WebGPU (react-native-webgpu).'
          : '2-D chart mode: WebGPU is not available in this build (Expo Go). Use a development build for 3D.'}
      </Text>
      <Text style={styles.h2}>Credits</Text>
      <Text style={styles.p}>
        Prior art: boatsim (whiskey.vodka). Models from Yasukawa & Yoshimura (2015), Savitsky (1964), Fossen's Handbook of Marine Craft Hydrodynamics and Motion
        Control, Kerner's water interaction model, DNV-RP-C205 wave spectra, and the 1€ filter (Casiez et al. 2012).
      </Text>
      <Text style={styles.link} onPress={() => Linking.openURL('https://github.com/coletebou/NautiSim')}>
        github.com/coletebou/NautiSim
      </Text>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, backgroundColor: colors.bg },
  content: { padding: 16, paddingBottom: 40 },
  h1: { color: colors.text, fontSize: 22, fontWeight: '800' },
  h2: { color: colors.text, fontSize: 17, fontWeight: '700', marginTop: 16 },
  p: { color: colors.text, fontSize: 14, lineHeight: 20, marginTop: 6, opacity: 0.9 },
  link: { color: colors.accent, fontSize: 15, marginTop: 16 },
});
