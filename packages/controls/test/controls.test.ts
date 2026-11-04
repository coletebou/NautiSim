import { describe, expect, it } from 'vitest';
import { ComplementaryFilter, LowPass, OneEuroFilter } from '../src/filters';
import { MadgwickIMU } from '../src/madgwick';
import { DEFAULT_MOTION_SETTINGS, dragWheel, InputMixer, mapGamepad, MotionHelm, type MotionSample, wheelToHelm } from '../src/mapping';
import { captureNeutral, deviceToScreen, quatFromDeviceRotation, rateLimit, relativeTilt, relativeTiltFromAttitude, shapeAxis, tiltFromGravity } from '../src/tilt';
import { qFromAxisAngle, qRotate, type V3 } from '../src/vec';

const DEG = Math.PI / 180;
const G = 9.81;

/**
 * Gravity in the device frame for a phone held in landscape (screen rotated `rotation`), tilted
 * back by `back`, turned like a steering wheel by `wheel` (clockwise positive) and with the top
 * edge pushed away by `push`. `sign` = +1 for a vector pointing at the ground (iOS style), −1 for
 * accelerometer reaction (Android style).
 */
function gravityFor(wheel: number, back: number, rotation: 0 | 90 | -90 | 180, sign: 1 | -1 = 1): V3 {
  // screen frame: start upright (down = −y), tilt back (screen faces up: down gains −z), then wheel
  const tb = back;
  let g = { x: 0, y: -Math.cos(tb), z: -Math.sin(tb) };
  // a clockwise wheel turn of the device rotates world vectors counter-clockwise in screen coords
  const c = Math.cos(wheel);
  const s = Math.sin(wheel);
  g = { x: c * g.x - s * g.y, y: s * g.x + c * g.y, z: g.z };
  // back from screen to device coordinates (inverse of deviceToScreen)
  const t = (rotation * Math.PI) / 180;
  const cr = Math.round(Math.cos(t));
  const sr = Math.round(Math.sin(t));
  const dev = { x: cr * g.x - sr * g.y, y: sr * g.x + cr * g.y, z: g.z };
  return { x: dev.x * G * sign, y: dev.y * G * sign, z: dev.z * G * sign };
}

describe('screen mapping', () => {
  it('deviceToScreen round-trips with the test helper for every rotation', () => {
    for (const r of [0, 90, -90, 180] as const) {
      const g = gravityFor(0.3, 0.5, r);
      const sv = deviceToScreen(g, r);
      const t = tiltFromGravity(sv);
      expect(t.roll).toBeCloseTo(0.3, 9);
    }
  });
});

describe('tilt extraction', () => {
  for (const sign of [1, -1] as const) {
    for (const r of [90, -90] as const) {
      it(`steering-wheel angle relative to neutral (rotation ${r}, gravity sign ${sign})`, () => {
        const neutral = captureNeutral([gravityFor(0, 40 * DEG, r, sign)], r);
        expect(neutral.gravitySign).toBe(sign);
        for (const wheel of [-30, -10, 0, 5, 25]) {
          const rel = relativeTilt(gravityFor(wheel * DEG, 40 * DEG, r, sign), neutral, r);
          expect(rel.roll / DEG).toBeCloseTo(wheel, 6);
          // turning the wheel must not register as pitch
          expect(Math.abs(rel.pitch)).toBeLessThan(1e-9);
        }
      });

      it(`pitch relative to neutral is decoupled from wheel angle (rotation ${r}, sign ${sign})`, () => {
        const neutral = captureNeutral([gravityFor(0, 40 * DEG, r, sign)], r);
        const rel = relativeTilt(gravityFor(20 * DEG, 55 * DEG, r, sign), neutral, r);
        expect(rel.roll / DEG).toBeCloseTo(20, 6);
        // tilting further back by 15° reads as −15° pitch (top edge away)
        expect(rel.pitch / DEG).toBeCloseTo(-15, 6);
      });
    }
  }

  it('fused attitude gives the same relative tilt as gravity', () => {
    const q0 = quatFromDeviceRotation(0.2, 0.7, 0.05);
    const turn = qFromAxisAngle({ x: 0, y: 0, z: 1 }, -0.3); // device turned clockwise about its screen normal
    const q1 = { w: 0, x: 0, y: 0, z: 0 };
    const m = (a: typeof q0, b: typeof q0) => ({
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    });
    Object.assign(q1, m(q0, turn));
    const r = relativeTiltFromAttitude(q1, q0, 0);
    expect(r.roll).toBeCloseTo(0.3, 6);
    expect(Math.abs(r.pitch)).toBeLessThan(1e-6);
  });
});

describe('axis shaping', () => {
  const curve = { deadzone: 2 * DEG, saturation: 30 * DEG, expo: 0 };
  it('dead zone rescales to start at zero and saturates at ±1', () => {
    expect(shapeAxis(1 * DEG, curve)).toBe(0);
    expect(shapeAxis(2.01 * DEG, curve)).toBeLessThan(0.01);
    expect(shapeAxis(16 * DEG, curve)).toBeCloseTo(0.5, 6);
    expect(shapeAxis(45 * DEG, curve)).toBe(1);
    expect(shapeAxis(-45 * DEG, curve)).toBe(-1);
    expect(shapeAxis(16 * DEG, { ...curve, invert: true })).toBeCloseTo(-0.5, 6);
  });
  it('expo gives finer control near centre and is monotonic', () => {
    const e = { ...curve, expo: 0.6 };
    expect(shapeAxis(16 * DEG, e)).toBeLessThan(0.5);
    let prev = -1;
    for (let a = -40; a <= 40; a += 0.5) {
      const v = shapeAxis(a * DEG, e);
      expect(v).toBeGreaterThanOrEqual(prev);
      prev = v;
    }
  });
  it('rate limit', () => {
    expect(rateLimit(0, 1, 2, 0.1)).toBeCloseTo(0.2, 12);
    expect(rateLimit(0.95, 1, 2, 0.1)).toBe(1);
  });
});

describe('filters', () => {
  it('low-pass reaches 63 % in one time constant', () => {
    const f = new LowPass(1);
    const tau = 1 / (2 * Math.PI);
    f.filter(0, 0);
    let y = 0;
    const dt = 0.0005;
    for (let t = 0; t < tau; t += dt) y = f.filter(1, dt);
    expect(y).toBeCloseTo(1 - Math.exp(-1), 2);
  });

  it('1€ filter: smooths jitter at rest but follows fast motion with little lag', () => {
    const f = new OneEuroFilter({ minCutoff: 1, beta: 0.5 });
    let maxJitter = 0;
    let seed = 1;
    const noise = (): number => {
      seed = (seed * 16807) % 2147483647;
      return (seed / 2147483647 - 0.5) * 0.02;
    };
    for (let i = 0; i < 600; i++) {
      const y = f.filter(noise(), 1 / 60);
      if (i > 60) maxJitter = Math.max(maxJitter, Math.abs(y));
    }
    expect(maxJitter).toBeLessThan(0.006);
    // a fast ramp: the lag stays small
    let last = 0;
    for (let i = 0; i < 30; i++) last = f.filter(i * 0.05, 1 / 60);
    expect(29 * 0.05 - last).toBeLessThan(0.15);
  });

  it('complementary filter rejects gyro bias drift', () => {
    const f = new ComplementaryFilter(0.5);
    let a = 0;
    for (let i = 0; i < 6000; i++) a = f.filter(0.2, 0.01, 0.01); // gyro bias 0.01 rad/s
    expect(a).toBeCloseTo(0.2, 1);
  });

  it('Madgwick converges to the accelerometer tilt and integrates the gyro', () => {
    const m = new MadgwickIMU(0.2);
    const tilt = 0.4;
    const up = { x: 0, y: Math.sin(tilt), z: Math.cos(tilt) }; // reaction vector, sensor rolled about x
    for (let i = 0; i < 4000; i++) m.update({ x: 0, y: 0, z: 0 }, up, 0.005);
    const zEarth = qRotate(m.q, up);
    expect(zEarth.z).toBeCloseTo(1, 3);
    // pure yaw rotation with consistent gravity
    const m2 = new MadgwickIMU(0.05);
    for (let i = 0; i < 1000; i++) m2.update({ x: 0, y: 0, z: 0.5 }, { x: 0, y: 0, z: 1 }, 0.002);
    const xAxis = qRotate(m2.q, { x: 1, y: 0, z: 0 });
    expect(Math.atan2(xAxis.y, xAxis.x)).toBeCloseTo(1.0, 2);
  });
});

describe('MotionHelm', () => {
  const sample = (t: number, wheelDeg: number, backDeg = 40, rotation: 90 | -90 = 90, sign: 1 | -1 = 1): MotionSample => ({
    t,
    gravity: gravityFor(wheelDeg * DEG, backDeg * DEG, rotation, sign),
    rotation,
  });

  it('auto-calibrates on the first samples and steers starboard when turned clockwise', () => {
    const h = new MotionHelm({ ...DEFAULT_MOTION_SETTINGS, helmRate: 0, smoothing: { minCutoff: 50, beta: 0 } });
    for (let i = 0; i < 10; i++) h.update(sample(i / 60, 0));
    let out = { helm: 0, throttle: 0, throttleActive: false };
    for (let i = 10; i < 120; i++) out = h.update(sample(i / 60, 20));
    expect(out.helm).toBeGreaterThan(0.4);
    for (let i = 120; i < 240; i++) out = h.update(sample(i / 60, -20));
    expect(out.helm).toBeLessThan(-0.4);
  });

  it('recalibrating at a new pose re-centres the helm', () => {
    const h = new MotionHelm({ ...DEFAULT_MOTION_SETTINGS, helmRate: 0 });
    for (let i = 0; i < 60; i++) h.update(sample(i / 60, 15));
    h.calibrate(90);
    let out = { helm: 1, throttle: 0, throttleActive: false };
    for (let i = 60; i < 120; i++) out = h.update(sample(i / 60, 15));
    expect(Math.abs(out.helm)).toBeLessThan(0.02);
  });

  it('pitch throttle: top edge away is ahead; astern can be disabled', () => {
    const h = new MotionHelm({ ...DEFAULT_MOTION_SETTINGS, throttleMode: 'pitch', helmRate: 0, smoothing: { minCutoff: 50, beta: 0 } });
    for (let i = 0; i < 10; i++) h.update(sample(i / 60, 0, 40));
    let out = h.update(sample(1, 0, 70));
    for (let i = 0; i < 30; i++) out = h.update(sample(1 + i / 60, 0, 70));
    expect(out.throttle).toBeGreaterThan(0.9);
    for (let i = 0; i < 30; i++) out = h.update(sample(2 + i / 60, 0, 15));
    expect(out.throttle).toBeLessThan(-0.5);
    h.settings = { ...h.settings, throttle: { ...h.settings.throttle, allowAstern: false } };
    out = h.update(sample(3, 0, 15));
    expect(out.throttle).toBe(0);
  });

  it('helm rate limit bounds how fast the rudder command moves', () => {
    const h = new MotionHelm({ ...DEFAULT_MOTION_SETTINGS, helmRate: 1, smoothing: { minCutoff: 50, beta: 0 } });
    for (let i = 0; i < 10; i++) h.update(sample(i / 60, 0));
    const out = h.update(sample(10 / 60 + 0.1, 40));
    expect(out.helm).toBeLessThanOrEqual(0.1 + 1 / 60 + 1e-9);
  });
});

describe('touch wheel, gamepad and mixing', () => {
  it('dragging clockwise around the wheel turns it to starboard', () => {
    const c = { x: 100, y: 100 };
    let w = { angle: 0 };
    // screen y grows downward: right (100,0 offset) to bottom is clockwise on screen
    w = dragWheel(w, c, { x: 200, y: 100 }, { x: 100, y: 200 }, Math.PI);
    expect(w.angle).toBeCloseTo(Math.PI / 2, 9);
    expect(wheelToHelm(w, Math.PI)).toBeCloseTo(0.5, 9);
    w = dragWheel(w, c, { x: 100, y: 200 }, { x: 0, y: 100 }, Math.PI);
    w = dragWheel(w, c, { x: 0, y: 100 }, { x: 100, y: 0 }, Math.PI);
    expect(w.angle).toBe(Math.PI); // clamped at the lock
  });

  it('gamepad dead zone and triggers', () => {
    expect(mapGamepad({ axes: [0.1], buttons: [] }).helm).toBe(0);
    expect(mapGamepad({ axes: [1], buttons: [] }).helm).toBeCloseTo(1, 9);
    const g = mapGamepad({ axes: [-0.56], buttons: [0, 0, 0, 0, 0, 0, 0.2, 0.9] });
    expect(g.helm).toBeLessThan(0);
    expect(g.throttle).toBeCloseTo(0.7, 9);
  });

  it('mixer: the most recently moved source takes over, then hands back after idling', () => {
    const m = new InputMixer('motion', 2);
    m.offer('motion', 'helm', 0.1, 0);
    m.offer('touch', 'helm', 0, 0);
    expect(m.read('helm', 0).source).toBe('motion');
    m.offer('touch', 'helm', 0.8, 1);
    expect(m.read('helm', 1).value).toBeCloseTo(0.8, 9);
    expect(m.read('helm', 1).source).toBe('touch');
    m.offer('touch', 'helm', 0.8, 4);
    expect(m.read('helm', 4).source).toBe('motion');
  });
});
