# Roadmap

## v0.1 (this release)

- [x] Deterministic 6-DOF engine: rigid body, RK4 and semi-implicit Euler, mesh hydrostatics on waves
- [x] KVLCC2 with the MMG standard method, validated against published turning and zig-zag results
- [x] Two planing boats with time-domain Savitsky lift, outboard / sterndrive propulsion
- [x] JONSWAP seas with spreading, gusting wind, current, depth model
- [x] Tilt-to-steer with calibration, curves, filtering; touch wheel and throttle lever
- [x] Expo app: WebGPU 3D view, chase / bridge / orbit cameras, HUD, settings, haptics, chart fallback
- [x] CI: typecheck, Biome, tests, expo-doctor, iOS and Android bundle export

## On-device checklist (needs a phone)

- [ ] Tilt steering direction and feel on iOS and Android, both landscape orientations (there is an "Invert steering" switch if a platform reports axes the other way)
- [ ] `DeviceMotion` gravity sign and `rotationRate` axis order on each platform (the tilt code auto-detects the gravity sign at calibration; the gyro-fusion option assumes the documented axis order)
- [ ] WebGPU renderer start-up, frame rate and thermal behaviour on a mid-range Android phone
- [ ] Physics cost per frame on device (target: under 4 ms for the small boats); tune the ocean grid and wave component count if needed
- [ ] Haptic intensity and rate

## Next

- **Performance:** cache the free-surface height on a small patch around the hull per step instead of per vertex; move the `SimHost` to a `react-native-worklets` worker runtime if profiling shows JS-thread contention.
- **Native sensor module:** an Expo Module (Swift CoreMotion / Kotlin SensorManager) that batches 100–200 Hz samples and runs the same filters natively, if JS-side delivery is too coarse.
- **Gamepads:** a native module over Apple's GameController framework and Android's InputDevice, feeding the existing `mapGamepad` and `InputMixer`.
- **Ocean rendering:** GPU-side waves (three.js TSL node material) and an FFT cascade for high-end devices; foam from wave steepness; spray at the bow of planing boats.
- **More physics:** shallow-water and bank effects for MMG, squat, Savitsky & Brown pre-planing resistance (replacing the calibrated hump), trim tabs and outboard trim, porpoising checks.
- **Sailing:** DSYHS bare-hull resistance plus a sail coefficient-table model.
- **More vessels:** harbour tug with azimuth thrusters, a container ship (KCS, also has public MMG data).
- **Autopilot and instruments:** heading-hold autopilot, radar-style chart, depth sounder.

## Later (optional layer)

- Voyages, ports, contracts and an economy (from the boatsim prior art), multiplayer, replays and ghost runs.
