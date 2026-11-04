# NautiSim architecture

This file records the design as Architecture Decision Records (ADRs): each one states the context,
the decision, why, and what we rejected. Background and sources are in [`research.md`](research.md).

## Overview

```
apps/mobile (Expo, expo-router)
  ├─ sim/        SimHost: fixed-step loop, input → physics → snapshot
  ├─ render/     three.js (WebGPU) scene: ocean grid, hull, wake, cameras
  ├─ hud/        React Native overlay: speed, heading, RPM, rudder, heel/trim, chart
  ├─ input/      DeviceMotion, touch helm, gamepad adapters → @nautisim/controls
  └─ app/        screens: home (hull picker), sim, settings (calibration), about
        │
        ▼ imports (TypeScript source, no build step)
packages/controls     pure: tilt extraction, neutral pose, dead zone, curves, 1€ / Madgwick filters,
                      touch and gamepad mapping, source arbitration → HelmCommand
packages/physics      pure: units, vec/quat math, 6-DOF rigid body, integrators, hydrostatics
                      (mesh clipping), force models (MMG, Savitsky, propellers, windage, damping),
                      hull definitions as data, Simulation (fixed step)
packages/environment  pure: seeded PRNG, JONSWAP + spreading, wave field, wind with gusts,
                      current, depth
```

The three packages have no dependency on React, React Native, the DOM, or wall-clock time. The
physics package depends on `environment` only through a small `EnvironmentSampler` interface, so it
can be driven by a calm-water stub in tests.

### Frames and units

- World frame: North-East-Down (NED), metres, origin at the scenario start.
- Body frame: x forward, y starboard, z down (SNAME/Fossen), origin at the centre of gravity.
- Attitude: unit quaternion rotating body vectors into NED.
- SI units everywhere inside the engine (m, s, kg, N, rad). Knots and degrees exist only at the
  UI edge, through helpers in `physics/units`. Public APIs use branded number types (`Meters`,
  `Radians`, …) where a mix-up is plausible.
- The renderer converts NED to three.js (x east, y up, z south) in one function.

---

## ADR-001: Monorepo with pnpm, TypeScript source packages

**Decision.** One pnpm workspace with `packages/*` and `apps/*`. Packages export their TypeScript
source directly (`"main": "src/index.ts"`); Metro, Vitest and `tsc` all consume source. pnpm uses
`node-linker=hoisted`, which Expo and Metro handle best.

**Why.** No build step for libraries keeps the edit-run loop instant and the stack small. Hoisting
avoids the remaining Metro edge cases with symlinked, isolated `node_modules`.

**Rejected.** Bun workspaces (Expo support is good but pnpm is the documented Expo monorepo path);
Nx/Turborepo (unnecessary at this size); building packages to `dist` (extra moving parts).

## ADR-002: Physics model stack

**Decision.** A 6-DOF rigid body with diagonal added mass, rigid-body Coriolis, and pluggable force
models summed at the centre of gravity:

| Force model                                                                                  | Used by       | Source                         |
| -------------------------------------------------------------------------------------------- | ------------- | ------------------------------ |
| Mesh hydrostatics (triangle clipping against the wave surface, i.e. nonlinear Froude–Krylov) | all hulls     | Kerner 2015                    |
| Linear + quadratic damping in heave/roll/pitch from target damping ratios                    | all hulls     | Fossen                         |
| MMG hull, propeller and rudder (3-DOF horizontal)                                            | KVLCC2        | Yasukawa & Yoshimura 2015      |
| Added-mass Coriolis and Munk moment (optional)                                               | planing hulls | Fossen                         |
| Savitsky dynamic lift and friction (time domain)                                             | RIB, runabout | Savitsky 1964                  |
| Propeller with quadratic KT/KQ and power-limited engine, vectored outboard thrust            | planing hulls | B-series style fit             |
| Manoeuvring cross-flow drag for planing hulls                                                | planing hulls | Fossen cross-flow              |
| Windage (frontal/lateral areas, centroid)                                                    | all hulls     | Isherwood-style simplification |

Hulls are data: a `HullDefinition` holds principal dimensions, mass properties, a geometry recipe,
and the list of force-model configurations. Factory functions turn that into a runtime `Vessel`.

**Why.** MMG is the only open, standardised model with published coefficients and free-running
validation data. Savitsky is the industry default for planing craft and is cheap enough to run every
step. Mesh hydrostatics gives correct heel, trim and wave response for any hull shape without a
seakeeping database.

**Planing model details.** Savitsky's equations describe steady planing of a prismatic hull, so
running them every step needs a few documented extensions, each kept small and labelled in code:
- the trim used for lift is the geometric trim relative to the local surface plus the angle from
  the bottom's vertical velocity (this is what damps heave and pitch);
- an added-mass-flux (Wagner-type) damping term, c = π/4 ρ V b², for heave and pitch, one-sided
  (full strength while the bottom slams in, 15 % on the way out) so it never pulls the stern down;
- vertical pressure drag on every wetted panel moving down into the water (Kerner's "pressure
  drag"), which is the distributed slamming resistance that keeps a boat from launching off waves;
- the forefoot ahead of the prismatic bottom gets its own Savitsky lift at its local buttock angle,
  which is what lifts the bow over the hump;
- heel is handled by evaluating the two halves of the V with deadrise β ∓ φ, which gives the
  dynamic roll stiffness, plus a small sideslip-asymmetry term;
- a pre-planing "hump" resistance and bow-up moment, **calibrated** (not from a published
  regression) until Savitsky & Brown (1976) lands;
- sway resistance from wetted-panel cross-flow drag on the actual mesh, so it still works when the
  hull is heeled onto its chine.

**Rejected.** Full strip theory or panel methods in real time (too slow on phones); a single generic
"arcade boat" model (cannot be validated against anything).

## ADR-003: Rendering stack: three.js on WebGPU via react-native-wgpu

**Decision.** Render with three.js's `WebGPURenderer` (`three/webgpu`) on a `react-native-wgpu`
canvas, driven imperatively (no React Three Fiber). The ocean is a CPU-displaced grid that follows
the camera and samples the same wave function as the physics. When the WebGPU native module is not
present (for example in Expo Go), the app falls back to a 2D chart view (react-native-svg) so the
physics, controls and HUD still run.

**Why.**
- WebGPU (Metal on iOS, Vulkan on Android, through Dawn) is where the React Native 3D ecosystem has
  moved; R3F maintainers explicitly push native users to WebGPU because of expo-gl's performance and
  OpenGL ES's deprecation on Apple platforms.
- Imperative three.js avoids a reconciler between React and the scene: the frame loop touches only
  plain objects, which matters on Hermes. R3F's native package is in a pre-release migration.
- Sampling the physics wave function for the visible ocean guarantees the hull sits on the waves
  the player sees. A few thousand vertices times eight wave components is a few hundred thousand
  trig calls per frame, well within a phone's budget.

**Rejected.** expo-gl + three WebGLRenderer (works, but slow and on a deprecated API on iOS);
React Three Fiber native (in flux); Skia (2D only); a WebView (adds a second JS engine and input
latency for tilt control); a GPU FFT ocean in v0.1 (needs compute shaders and TSL work that cannot be
verified without a device; on the roadmap).

**Consequences.** The 3D view requires a development build (`expo run:*` or EAS Build). That is
stated plainly in the README.

## ADR-004: Threading: physics off the UI thread, behind a SimHost interface

**Decision.** A `SimHost` owns the fixed-step accumulator. In v0.1 it runs on the React Native JS
thread inside the render loop callback, which is separate from the platform UI (main) thread, so
touch handling and native animations never wait for physics. React state is not touched per frame:
the HUD reads a snapshot store at 10–15 Hz. The physics packages are pure functions over plain
data so the host can move to a dedicated Worker Runtime later (`react-native-worklets`
`createWorkletRuntime` + `scheduleOnRuntime`).

**Why.** The renderer (three.js) and the sensor events (expo-sensors) both live on the JS thread
today, so a Worker Runtime would add a copy of the state every frame in both directions and a
thread hop on the sensor path. Measured budget on desktop Node: a KVLCC2 RK4 step with full mesh
hydrostatics costs well under a millisecond; a phone is perhaps 5–10× slower, still inside a 60 Hz
frame. The interface keeps the move cheap if profiling on real devices says otherwise.

**Rejected.** Physics on the UI runtime via Reanimated (`useFrameCallback`): it would compete with
gesture handling, which is exactly what we want to avoid. A native C++ physics core: premature.

## ADR-005: Integration and determinism

**Decision.** Fixed step (1/60 s for small craft, 1/30 s for the full-scale tanker), accumulator with a
maximum of 8 sub-steps per frame (to avoid a spiral of death), render interpolation between the last
two states, RK4 by default with semi-implicit Euler available. Quaternion renormalised after every
step. Seeded PRNG only; the step function takes time from the step counter, never from the clock.

**Why.** Reproducible tests and replays, stable at phone frame rates, and standard practice
(Fiedler, _Fix Your Timestep!_).

## ADR-006: Device motion input

**Decision.** Default steering source is the gravity vector from `DeviceMotion`
(`accelerationIncludingGravity`), turned into a "steering-wheel" angle and a pitch angle relative to
a captured neutral pose, corrected for landscape-left/right. Optional sources: the fused attitude
quaternion from `DeviceMotion.rotation`, and a 6-axis Madgwick fusion of raw gyro + accelerometer in
JS. All filtering (1€, low-pass) and mapping (dead zone, saturation angle, expo curve, rate limit,
invert) is in `@nautisim/controls` and unit tested. Sensors run at 60 Hz by default.

**Why.** Gravity is measured the same way on iOS and Android and does not drift, so it avoids the
platform differences in Euler angle conventions and magnetometer use. The fused attitude is better
for fast motion and is offered as an option.

**Fallback plan.** If JS-side sensor delivery proves too coarse or jittery on device, write a small
Expo Module (Swift CoreMotion / Kotlin SensorManager) that batches samples at 100–200 Hz and runs
the same filters natively. The pure TypeScript implementation remains the reference.

## ADR-007: Tooling

**Decision.** Strict TypeScript 6 (the version Expo SDK 58 expects; `strict`, `noUncheckedIndexedAccess`,
`exactOptionalPropertyTypes` in the pure packages), Vitest for tests, Biome for lint + format (one fast tool instead of
ESLint + Prettier), GitHub Actions running typecheck, lint, tests, `expo export` for iOS and
Android, and `expo-doctor`. `eas.json` with development, preview and production profiles and no
secrets. Conventional Commits.

## ADR-008: Scope of v0.1

In: three hulls (KVLCC2 tanker, 6.5 m RIB, 8 m sport runabout), calm water and JONSWAP seas, wind
and current, tilt/touch helm, chase and bridge cameras, HUD, settings with calibration, haptics.

Out (roadmap): sailing (DSYHS + sail tables), shallow-water and bank effects, squat, voyages and
contracts, multiplayer, GPU FFT ocean, native gamepad module, native sensor module.
