/**
 * Typed units. Inside the engine everything is SI (m, s, kg, N, rad); these brands exist so that
 * public APIs cannot silently mix knots with m/s or degrees with radians. A brand is a compile-time
 * tag only: at runtime the values are plain numbers and cost nothing.
 */
declare const brand: unique symbol;
export type Brand<T, B extends string> = T & { readonly [brand]: B };

export type Meters = Brand<number, 'm'>;
export type Seconds = Brand<number, 's'>;
export type Kilograms = Brand<number, 'kg'>;
export type Newtons = Brand<number, 'N'>;
export type Radians = Brand<number, 'rad'>;
export type Degrees = Brand<number, 'deg'>;
export type MetersPerSecond = Brand<number, 'm/s'>;
export type Knots = Brand<number, 'kn'>;
export type RadiansPerSecond = Brand<number, 'rad/s'>;
export type Watts = Brand<number, 'W'>;
export type RevsPerSecond = Brand<number, 'rps'>;

export const m = (v: number): Meters => v as Meters;
export const s = (v: number): Seconds => v as Seconds;
export const kg = (v: number): Kilograms => v as Kilograms;
export const N = (v: number): Newtons => v as Newtons;
export const rad = (v: number): Radians => v as Radians;
export const deg = (v: number): Degrees => v as Degrees;
export const mps = (v: number): MetersPerSecond => v as MetersPerSecond;
export const kn = (v: number): Knots => v as Knots;
export const W = (v: number): Watts => v as Watts;

/** Exact conversion factors. */
export const KNOT_IN_MPS = 1852 / 3600;
export const DEG_IN_RAD = Math.PI / 180;
export const KW_IN_W = 1000;
export const HP_IN_W = 745.699872;

export const knotsToMps = (v: Knots | number): MetersPerSecond => (v * KNOT_IN_MPS) as MetersPerSecond;
export const mpsToKnots = (v: MetersPerSecond | number): Knots => (v / KNOT_IN_MPS) as Knots;
export const degToRad = (v: Degrees | number): Radians => (v * DEG_IN_RAD) as Radians;
export const radToDeg = (v: Radians | number): Degrees => (v / DEG_IN_RAD) as Degrees;
export const rpsToRpm = (v: number): number => v * 60;

/** Standard physical constants used as defaults. */
export const GRAVITY = 9.80665;
export const RHO_SEAWATER = 1025;
export const RHO_FRESHWATER = 999.1;
export const NU_SEAWATER_15C = 1.188e-6;
export const RHO_AIR = 1.225;
