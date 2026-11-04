# NautiSim research notes

Status: November 2026. This document collects the state of the art that NautiSim's design rests
on. Every section ends with what we take from it. Decisions themselves are recorded in
[`architecture.md`](architecture.md).

Plain-language glossary for the terms used below:

- **DOF (degree of freedom)**: one independent way a body can move. A boat has six: surge
  (forward/back), sway (sideways), heave (up/down), roll (heel side to side), pitch (bow up/down)
  and yaw (turning).
- **Hydrodynamic derivative**: a measured coefficient that says how much force the water puts on
  the hull for a given motion, for example "this much side force per unit of sideways speed".
- **Captive model test**: a towing-tank test where a scale model is held on a carriage and forced
  through motions while the forces are measured.
- **Free-running test**: the scale model drives itself with its own propeller and rudder, which is
  what the simulator should reproduce.
- **Froude number (Fn)**: speed divided by the square root of (gravity times length). Two boats at the
  same Froude number make geometrically similar waves, which is how model tests scale to full size.
- **Spectrum**: a recipe that says how much wave energy sits at each wave period; summing many
  waves drawn from it gives a realistic irregular sea.

---

## 1. Ship maneuvering: the MMG standard method

The Japanese Maneuvering Modeling Group (MMG) model splits the horizontal-plane forces on a ship
into hull, propeller and rudder parts and couples them through a small set of interaction factors
(wake fraction, thrust deduction, flow straightening, rudder-force increase). The JASNAOE committee
standardised it and published the full model, test procedure and a worked data set for the KVLCC2
tanker.

- Yasukawa, H. and Yoshimura, Y. (2015). *Introduction of MMG standard method for ship maneuvering
  predictions.* Journal of Marine Science and Technology 20, 37–52. Open access:
  <https://link.springer.com/article/10.1007/s00773-014-0293-y>
  - Hull forces are third-order polynomials in non-dimensional sway velocity v′ and yaw rate r′
    (X′vv … N′rrr, Table 3).
  - Propeller: KT(J) = k0 + k1 J + k2 J², with (k0, k1, k2) = (0.2931, −0.2753, −0.1385) for KVLCC2;
    effective wake wP0 = 0.40 at model scale and 0.35 full scale; wake changes in maneuvering via
    C1 = 2.0 and C2 = 1.6 / 1.1.
  - Rudder: normal force FN = ½ ρ AR UR² fα sin αR with Fujii's lift slope fα = 6.13Λ/(Λ+2.25) = 2.747,
    flow straightening γR = 0.395 / 0.640, lR′ = −0.710, ε = 1.09, κ = 0.50, tR = 0.387, aH = 0.312,
    xH′ = −0.464.
  - Added masses from Motora's charts: m′x = 0.022, m′y = 0.223, J′z = 0.011 (by ½ρL²d and ½ρL⁴d).
  - Simulations at Fn = 0.142 (15.5 kn full scale, 1.179 m/s for the 7 m L7 model).
  - Computed turning indices are within about 5.8 % of free-running tests; computed overshoot angles
    are smaller than measured (about 3° for the 10/10 first overshoot, 6° for the second).
- The open-source Python package ShipMMG implements the same method and lists the same KVLCC2 L7
  parameters (Lpp 7.00 m, B 1.27 m, d 0.46 m, ∇ 3.27 m³, xG 0.25 m, DP 0.216 m, HR 0.345 m,
  AR 0.0539 m²): <https://github.com/ShipMMG/shipmmg>,
  <https://gist.github.com/taiga4112/e53e28053784fef407692becc8226735>
- Reference results for validation (L7 model, Fn 0.142), tabulated by Wang et al. in Polish Maritime
  Research from MARIN's free-running tests and Yasukawa's MMG computation:
  <https://journal.mostwiedzy.pl/pmr/article/download/421/353/483>

  | Manoeuvre | Quantity | MARIN free-running | Yasukawa MMG |
  | --- | --- | --- | --- |
  | +35° turn | advance / Lpp | 3.25 | 3.31 |
  | +35° turn | tactical diameter / Lpp | 3.34 | 3.36 |
  | −35° turn | advance / Lpp | 3.11 | 3.26 |
  | −35° turn | tactical diameter / Lpp | 3.08 | 3.26 |
  | 10°/10° zig-zag | 1st overshoot (°) | 8.2 | 5.2 |
  | 10°/10° zig-zag | 2nd overshoot (°) | 21.9 | 15.8 |
  | 20°/20° zig-zag | 1st overshoot (°) | 13.7 | 10.9 |
  | 20°/20° zig-zag | 2nd overshoot (°) | 14.8 | 16.8 |

- The MARIN uncertainty paper (Quadvlieg et al., "KVLCC2 benchmark data including uncertainty
  analysis to support manoeuvring predictions", MARINE 2011, <http://hdl.handle.net/2117/333239>)
  quantifies repeat-test scatter: about ±0.1 Lpp on advance and tactical diameter and about ±1° on
  overshoot angles. The ITTC procedure 7.5-02-06-05 explains the overshoot uncertainty sources
  (<https://ittc.info/media/4142/75-02-06-05.pdf>).
- MANSIM's report (<https://mansim.org/articles/Theoretical%20background%20and%20application%20of%20MANSIM%20for%20ship%20maneuvering%20simulations.pdf>)
  shows the same full-scale comparison and notes that zig-zag results are very sensitive to initial
  conditions and rudder execution timing.

**What we take:** MMG with the published KVLCC2 coefficients is the validation anchor for the
displacement-ship path. Our simulator implements the same equations, so the primary test is "we
reproduce Yasukawa's MMG numbers closely", and the secondary test is "we land inside the
free-running scatter band". Overshoot angles get wider tolerances than turning geometry because the
literature itself shows that.

## 2. Fossen's 6-DOF vessel model and the MSS toolbox

Fossen's matrix-vector form writes the equations of motion as
M ν̇ + C(ν) ν + D(ν) ν + g(η) = τ, with rigid-body and added-mass inertia in M, Coriolis and
centripetal terms in C, damping in D, restoring (buoyancy and gravity) in g, and everything else
(propulsion, wind, waves) in τ. Velocities ν = [u, v, w, p, q, r] are in the body frame, positions
η in a North-East-Down (NED) world frame, following SNAME naming.

- Fossen's model overview: <https://www.fossen.biz/html/marineCraftModel.html>
- Marine Systems Simulator (MSS), MATLAB/Octave, release 2.0 with a generic 12-state 6-DOF vessel
  model, JONSWAP waves with directional spreading and first-order wave loads:
  <https://github.com/cybergalactic/MSS>, <https://fossen.biz/MSS/>
- Fossen, T. I. *Handbook of Marine Craft Hydrodynamics and Motion Control* (Wiley, 2nd ed. 2021;
  3rd ed. in progress).

**What we take:** body-frame state, NED world frame, SNAME symbols, the C(ν) split (rigid-body
Coriolis always on; added-mass Coriolis optional per hull, because MMG's captive-test derivatives
already contain the Munk moment and must not count it twice).

## 3. Planing hulls: Savitsky and extensions

- Savitsky, D. (1964). *Hydrodynamic Design of Planing Hulls.* Marine Technology 1(1), 71–95.
  <https://doi.org/10.5957/mt1.1964.1.4.71>. Empirical equations for a prismatic planing surface:
  - lift of a flat plate C_L0 = τ^1.1 (0.0120 λ^0.5 + 0.0055 λ^2.5 / Cv²), τ in degrees;
  - deadrise correction C_Lβ = C_L0 − 0.0065 β C_L0^0.6;
  - centre of pressure lp/λb = 0.75 − 1/(5.21 Cv²/λ² + 2.39);
  - mean bottom velocity, wetted keel and chine lengths, porpoising limits.
  The first lift term is dynamic lift; the second (with 1/Cv²) is the hydrostatic part.
- A modern worked example with intermediate values (30 kn, τ 4°, b 3.2 m, λ 1.71875, β 18°,
  7.4552 t: C_L0 = 0.085181, C_Lβ = 0.058488, Rf = 5.859 kN, Rp = 5.112 kN, RT = 10.957 kN):
  <https://nauticalsolver.com/calculators/resistance/savitsky/savitsky.php>
- USNA EN486 notes on planing resistance and the history of the lift equations:
  <https://usna.edu/NAOE/_files/documents/Courses/EN486/03_-_Resistance.pdf>
- Extensions in the literature: Savitsky, DeLorme and Datla (2007) on whisker spray drag; Savitsky
  and Brown (1976) on pre-planing; Zarnick (1978) strip theory for planing in waves; Faltinsen's
  *Hydrodynamics of High-Speed Marine Vehicles* (2005).

**What we take:** a time-domain Savitsky force model. Instantaneous trim, keel wetted length and
speed come from the 6-DOF state; only the dynamic lift term is applied (buoyancy already comes from
the mesh), acting normal to the planing bottom at Savitsky's centre of pressure. Running trim and
resistance then emerge from the dynamics instead of being prescribed. A static Savitsky solver is
kept for validation against the worked example.

## 4. Sailing yacht resistance and sail forces (roadmap)

- Delft Systematic Yacht Hull Series (DSYHS): Keuning and Katgert (2008), *A bare hull resistance
  prediction method derived from the results of the Delft Systematic Yacht Hull Series extended to
  higher speeds*, RINA, <https://research.tudelft.nl/en/publications/a-bare-hull-resistance-prediction-method-derived-from-the-results/>.
  The raw data is public (<http://dsyhs.tudelft.nl>); Remmlinger's re-regression
  (<https://remmlinger.com/Regression%20DSYHS.pdf>) reports lower error than the original fit.
- Sail forces in real-time sims are usually lift/drag coefficient tables versus apparent wind angle
  (the ORC VPP approach; Hazen's 1980 model; Fossati's *Aero-Hydrodynamics and the Performance of
  Sailing Yachts*, 2009).

**What we take:** the force-model interface is designed so a DSYHS resistance model and a
coefficient-table sail model drop in later. Sailing is on the roadmap, not in v0.1.

## 5. Waves: spectra, spreading, and what fits in real time

- JONSWAP (Hasselmann et al., 1973) is the standard fetch-limited wind-sea spectrum; it is the
  Pierson-Moskowitz spectrum times a peak-enhancement factor γ^r. DNV-RP-C205 gives the
  recommended γ as a function of Tp/√Hs (γ = 5 below 3.6, exp(5.75 − 1.15 Tp/√Hs) up to 5, and 1
  above). A practitioner summary (Bentley MOSES theory manual):
  <https://docs.bentley.com/LiveContent/web/MOSES-v2026/Theory%20Manual/en/env_sea_spect.html>.
  DNV-RP-C205 also gives the cos^2s and cos^n directional spreading functions.
- Linear (Airy) components with dispersion ω² = g k tanh(k h) are adequate for ship-scale
  seakeeping in a simulator; Gerstner/trochoidal displacement adds visual sharpening of crests.
- Real-time rendering on mobile:
  - Tessendorf FFT oceans are the film/AAA standard. ARM measured a GPU FFT pipeline at about
    2.2 ms per frame on a Mali-T760 (Galaxy S6) using compute shaders and FP16
    (<https://armkeil.blob.core.windows.net/developer/Files/pdf/graphics-and-multimedia/Realtime_GPU_ocean_rendering.pdf>).
  - A 2024 study (<https://www.mdpi.com/2077-1312/12/4/572>) argues FFT is best kept to consoles/PC
    and shows filtered Gerstner sums give comparable visuals on constrained platforms at a fraction
    of the cost.
  - A sum of a few dozen Gerstner waves is equivalent to a very sparse FFT
    (<https://rtryan98.github.io/2025/10/04/ocean-rendering-part-1.html>).

**What we take:** the environment package draws seeded, deterministic JONSWAP components with cos-2s
spreading. Physics samples the exact same function the renderer draws, so the boat floats on the
waves the user sees. v0.1 renders with a CPU-displaced grid around the camera (simple and identical
to the physics surface); a GPU (TSL compute) FFT cascade is on the roadmap for high-end devices.

## 6. Buoyancy from the hull mesh (game-physics approach)

- Jacques Kerner, *Water interaction model for boats in video games* (Game Developer, 2015) and
  part 2 (2016): approximate the hull with triangles, compute each vertex's height above the water,
  clip each triangle against the surface into 0–2 fully submerged triangles, then integrate
  hydrostatic pressure ρ g h n A over them. Part 2 adds viscous resistance, pressure drag and
  slamming per triangle.
  <https://www.gamedeveloper.com/programming/water-interaction-model-for-boats-in-video-games>,
  <https://www.gamedeveloper.com/programming/water-interaction-model-for-boats-in-video-games-part-2>
- Habrador's Unity tutorial is a readable implementation of the same idea:
  <https://www.habrador.com/tutorials/unity-boat-tutorial/3-buoyancy/>

Integrating hydrostatic pressure of the *incident* wave over the instantaneous wetted surface is
exactly the nonlinear Froude–Krylov force, which is the dominant wave load for a small boat and a
good-enough first-order load for a ship in a simulator. Radiation and diffraction are approximated
by added mass and linear damping tuned to a target damping ratio.

**What we take:** a triangle-clipping hydrostatics kernel with exact (not just vertical) pressure
integration, so moments and righting arms come out right, and a test suite that checks it against
closed-form box-barge hydrostatics (volume, KB, BM = B²/12T, GM).

## 7. Real-time integration and determinism

- Glenn Fiedler, *Fix Your Timestep!* (<https://gafferongames.com/post/fix_your_timestep/>): run
  physics at a fixed step from an accumulator, and interpolate render state between the last two
  steps.
- Quaternion attitude integration: q̇ = ½ q ⊗ (0, ω). RK4 on the full state with renormalisation
  is cheap and accurate for smooth marine dynamics; semi-implicit (symplectic) Euler is cheaper and
  stable for stiff contact but needs a smaller step. Exponential-map updates
  (q ← q ⊗ exp(½ ω dt)) keep unit length exactly.
- Determinism: the same seed and the same input stream must give bit-identical results on the same
  JS engine. That means no `Math.random`, no wall-clock reads inside the step, fixed iteration
  order, and a seeded PRNG (we use a 32-bit SplitMix/Mulberry style generator).

**What we take:** a fixed-step accumulator with interpolation, RK4 by default and semi-implicit
Euler as an option, quaternion renormalisation per step, seeded PRNG only.

## 8. Expo and React Native state of the art (late 2026)

- **Expo SDK 58** (released autumn 2026) ships React Native 0.88 and React 19.3; the New
  Architecture is the only architecture, and the strict TypeScript API is now the default
  (<https://expo.dev/changelog/sdk-58>, <https://expo.dev/blog/how-to-upgrade-to-sdk-58>,
  <https://reactnative.dev/blog/2026/10/19/react-native-0.88>, <https://docs.expo.dev/versions/v58.0.0/>).
- **Sensors.** `expo-sensors` DeviceMotion gives acceleration, acceleration including gravity,
  rotation (alpha/beta/gamma), rotationRate and screen orientation
  (<https://docs.expo.dev/versions/latest/sdk/devicemotion/>). Android 12+ caps sensor updates at
  200 Hz unless `HIGH_SAMPLING_RATE_SENSORS` is declared. On iOS the attitude comes from CoreMotion
  with the `XMagneticNorthZVertical` frame when a magnetometer exists
  (<https://github.com/expo/expo/pull/23738>); on Android it comes from `TYPE_ROTATION_VECTOR`.
  Axes and units have historically differed between platforms
  (<https://github.com/expo/expo/issues/7877>, <https://github.com/expo/expo/discussions/7893>), so
  we derive tilt from the gravity vector, which is consistent on both, and treat attitude angles as
  an optional source.
- **Threads.** `react-native-worklets` 0.13 (Software Mansion) can create Worker Runtimes on their
  own threads with `createWorkletRuntime` and schedule work with `scheduleOnRuntime`
  (<https://docs.swmansion.com/react-native-worklets/docs/threading/createWorkletRuntime/>,
  <https://docs.swmansion.com/react-native-worklets/docs/fundamentals/runtimeKinds/>). Worker
  runtimes share no memory with the main JS runtime.
- **3D graphics.**
  - `react-native-wgpu` (William Candillon) gives WebGPU on iOS and Android via Google's Dawn,
    with an Expo config plugin and documented three.js (`three/webgpu`) integration
    (<https://wcandillon.github.io/react-native-webgpu/docs/getting-started/expo>,
    <https://wcandillon.github.io/react-native-webgpu/docs/integrations/three-js>). It needs a
    development build (not Expo Go).
  - `expo-gl` (OpenGL ES) still works, but React Three Fiber maintainers describe "significant
    performance issues with ExpoGL" and are moving native support to WebGPU
    (<https://github.com/pmndrs/native>); OpenGL ES is deprecated on Apple platforms and unreliable
    on the iOS simulator (<https://github.com/pmndrs/react-three-fiber/pull/3572>).
  - React Three Fiber v10 (alpha) renders with WebGPU and has moved React Native support into a
    separate, not-yet-published package (<https://r3f.docs.pmnd.rs/next/migration/breaking-changes>).
  - Skia (`@shopify/react-native-skia`) is excellent for 2D (HUD dials, charts) but not a 3D engine.
- **Haptics.** `expo-haptics` (impact, notification, selection) on both platforms.
- **Game controllers.** React Native has no built-in gamepad API. Options are a native module over
  Apple's GameController framework and Android's InputDevice/MotionEvent, or community modules
  (for example `@thegreatzeus/expo-keyevent` for iOS buttons). A good integration normalises axes
  and dead zones natively and emits compact events to JS
  (<https://reactnative.live/gamepad-support-and-custom-controls-enhancing-react-native-a>).
- **Native modules.** The Expo Modules API (Swift/Kotlin over JSI) is the recommended way to add a
  CoreMotion/SensorManager module if the JS sensor path proves too coarse
  (<https://docs.expo.dev/modules/get-started/>, <https://docs.expo.dev/modules/module-api>).
- **Build and delivery.** EAS Build for store binaries and EAS Update for over-the-air JS updates;
  `eas.json` holds profiles and no secrets.

**What we take:** see ADR-003 (rendering), ADR-004 (threading) and ADR-006 (sensors) in
[`architecture.md`](architecture.md).

## 9. Motion-control UX

- Tilt-to-steer works best like a steering wheel: hold the phone in landscape and rotate it about
  the axis through the screen. The angle is read from the gravity vector projected into the screen
  plane, relative to a captured neutral pose.
- **Neutral-pose capture** lets players hold the phone at any comfortable angle; re-centre on demand
  and on resume.
- **Dead zone** with rescaling (so output starts from 0 at the dead-zone edge, not with a jump),
  **saturation angle** (how far to tilt for full helm), **expo / sensitivity curve**, optional
  **invert**, and **rate limiting** so a twitch cannot slam the rudder.
- **Filtering.**
  - The 1€ filter (Casiez, Roussel and Vogel, CHI 2012) is a speed-adaptive low-pass filter: low
    cutoff (little jitter) when the signal is still, higher cutoff (little lag) when it moves.
    Two parameters, easy tuning. <https://gery.casiez.net/1euro/>,
    <https://dl.acm.org/doi/10.1145/2207676.2208639>
  - Complementary filters fuse gyro rate (smooth, drifts) with accelerometer gravity (noisy, no
    drift). Madgwick's gradient-descent filter (2010) does this on quaternions at very low cost
    (<https://x-io.co.uk/downloads/madgwick_internal_report.pdf>).
- **Orientation handling.** Swap and sign-flip the screen axes for landscape-left and
  landscape-right, and re-capture neutral when the orientation flips.
- **Accessibility fallbacks.** On-screen wheel and throttle lever, gamepad, a "hold to steer"
  mode, adjustable sensitivity, and haptics that can be turned off.

**What we take:** the `controls` package implements all of the above as pure functions with unit
tests; the app wires them to DeviceMotion, touch and (later) gamepads.

## 10. Prior art: boatsim

NautiSim follows boatsim (<https://whiskey.vodka/boatsim/>), a browser bridge simulator by the
same owner. Its source showed which models are worth the cost in a real-time sim: MMG for big
ships, Savitsky for planing craft, mesh-clipped hydrostatics for everything, B-series propellers,
Isherwood-style windage and Barrass squat. NautiSim keeps the same choice of models but is a
fresh, smaller codebase with a stricter separation between pure physics, environment, input and
rendering, and it targets native mobile instead of the browser.
