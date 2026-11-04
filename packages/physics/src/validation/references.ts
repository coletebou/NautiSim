/**
 * Published reference numbers used by the validation tests. Each block names its source.
 */

/**
 * KVLCC2 L7 model (λ = 45.714, Lpp = 7.0 m) at Fn = 0.142 (U0 = 1.179 m/s). Free-running results
 * from MARIN and the MMG computation of Yasukawa & Yoshimura (2015), as tabulated by Wang et al.,
 * Polish Maritime Research (https://journal.mostwiedzy.pl/pmr/article/download/421/353/483).
 * Overshoot angles in degrees, distances by Lpp.
 */
export const KVLCC2_L7_REFERENCE = {
  approachSpeed: 1.179,
  /**
   * Model self-propulsion point: 9.9 rps for the HMRI KVLCC2 model at λ = 46.426
   * (http://simman2020.kr/download/KVLCC2-Propeller-Openwater-Data-HMRI.pdf), Froude-scaled to the
   * L7 model: n ∝ √λ.
   */
  selfPropulsionRps: 9.9 * Math.sqrt(46.426 / 45.714),
  turning: {
    starboard: { marin: { advance: 3.25, tacticalDiameter: 3.34 }, mmg: { advance: 3.31, tacticalDiameter: 3.36 } },
    port: { marin: { advance: 3.11, tacticalDiameter: 3.08 }, mmg: { advance: 3.26, tacticalDiameter: 3.26 } },
  },
  zigzag: {
    '10/10': { marin: [8.2, 21.9], mmg: [5.2, 15.8] },
    '-10/-10': { marin: [9.5, 15.0], mmg: [7.6, 10.2] },
    '20/20': { marin: [13.7, 14.8], mmg: [10.9, 16.8] },
    '-20/-20': { marin: [15.1, 13.2], mmg: [14.5, 12.4] },
  },
} as const;

/**
 * Savitsky (1964) worked example as computed by the Nautical Solver calculator
 * (https://nauticalsolver.com/calculators/resistance/savitsky/savitsky.php): 30 kn, trim 4°,
 * chine beam 3.2 m, λ = 1.71875, deadrise 18°, 7.4552 t, ρ = 1025, ν = 1.19e-6, ΔCf = 0.0004.
 */
export const SAVITSKY_EXAMPLE = {
  speedKnots: 30,
  trimDeg: 4,
  beam: 3.2,
  lambda: 1.71875,
  deadriseDeg: 18,
  massTonnes: 7.4552,
  rho: 1025,
  nu: 1.19e-6,
  deltaCf: 0.0004,
  expected: {
    cv: 2.755,
    cl0: 0.085181,
    clBeta: 0.058488,
    liftN: 73_111,
    meanVelocity: 15.215,
    cf: 0.002594,
    frictionN: 5_859,
    pressureDragN: 5_112,
    totalN: 10_957,
  },
} as const;
