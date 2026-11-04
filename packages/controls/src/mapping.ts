import { ComplementaryFilter, OneEuroFilter } from './filters';
import { MadgwickIMU } from './madgwick';
import { type AxisCurve, captureNeutral, type NeutralPose, rateLimit, relativeTilt, relativeTiltFromAttitude, type ScreenRotation, shapeAxis } from './tilt';
import { clamp, type Q4, type V3 } from './vec';

/** Normalised operator command, the output of the controls package. */
export interface HelmCommand {
  /** −1 (hard to port) … 1 (hard to starboard). */
  helm: number;
  /** −1 (full astern) … 1 (full ahead). */
  throttle: number;
}

export type TiltSource = 'gravity' | 'attitude' | 'fusion';
export type ThrottleMode = 'pitch' | 'slider';

export interface MotionSettings {
  source: TiltSource;
  steering: AxisCurve;
  throttleMode: ThrottleMode;
  /** Pitch toward you (top edge back) for ahead, away for astern. */
  throttle: AxisCurve & { allowAstern: boolean };
  /** Max helm change per second (1/s); 0 disables the limit. */
  helmRate: number;
  /** 1€ filter parameters for the steering angle. */
  smoothing: { minCutoff: number; beta: number };
}

export const DEFAULT_MOTION_SETTINGS: MotionSettings = {
  source: 'gravity',
  steering: { deadzone: (2 * Math.PI) / 180, saturation: (35 * Math.PI) / 180, expo: 0.35 },
  throttleMode: 'slider',
  throttle: { deadzone: (5 * Math.PI) / 180, saturation: (30 * Math.PI) / 180, expo: 0.2, allowAstern: true },
  helmRate: 3,
  smoothing: { minCutoff: 1.2, beta: 0.6 },
};

/** One motion sample in device coordinates (as delivered by expo-sensors DeviceMotion). */
export interface MotionSample {
  /** Seconds, monotonic. */
  t: number;
  /** Acceleration including gravity, device frame (m/s²). */
  gravity: V3;
  /** Rotation rate, device frame (rad/s), when available. */
  gyro?: V3;
  /** Fused attitude, device → earth, when available. */
  attitude?: Q4;
  rotation: ScreenRotation;
}

/**
 * Turns a stream of motion samples into helm (and optionally throttle) commands: filtering,
 * neutral-pose calibration, curve shaping and rate limiting. Pure logic, no sensor APIs.
 */
export class MotionHelm {
  settings: MotionSettings;
  neutral: NeutralPose | undefined;
  private readonly rollFilter: OneEuroFilter;
  private readonly pitchFilter: OneEuroFilter;
  private readonly fusion = new MadgwickIMU(0.08);
  private readonly rollComp = new ComplementaryFilter(0.3);
  private lastT: number | undefined;
  private helm = 0;
  private window: V3[] = [];
  private lastAttitude: Q4 | undefined;
  /** Last relative tilt (rad) after filtering, for the calibration UI. */
  readonly tilt = { roll: 0, pitch: 0, planarity: 1 };

  constructor(settings: MotionSettings = DEFAULT_MOTION_SETTINGS) {
    this.settings = settings;
    this.rollFilter = new OneEuroFilter(settings.smoothing);
    this.pitchFilter = new OneEuroFilter(settings.smoothing);
  }

  /** Capture the current pose as neutral (averages the last ~0.3 s of samples). */
  calibrate(rotation: ScreenRotation): NeutralPose {
    this.neutral = {
      ...captureNeutral(this.window, rotation),
      ...(this.lastAttitude !== undefined && { attitude: this.lastAttitude }),
    };
    this.rollFilter.reset();
    this.pitchFilter.reset();
    this.rollComp.reset();
    this.helm = 0;
    return this.neutral;
  }

  update(s: MotionSample): HelmCommand & { throttleActive: boolean } {
    const dt = this.lastT === undefined ? 0 : clamp(s.t - this.lastT, 0, 0.25);
    this.lastT = s.t;
    this.window.push(s.gravity);
    if (this.window.length > 20) this.window.shift();
    if (s.attitude) this.lastAttitude = s.attitude;
    if (!this.neutral || this.neutral.rotation !== s.rotation) this.calibrate(s.rotation);
    const neutral = this.neutral as NeutralPose;
    let roll: number;
    let pitch: number;
    let planarity = 1;
    const src = this.settings.source;
    if (src === 'attitude' && s.attitude && neutral.attitude) {
      const r = relativeTiltFromAttitude(s.attitude, neutral.attitude, s.rotation);
      roll = r.roll;
      pitch = r.pitch;
    } else {
      const r = relativeTilt(s.gravity, neutral, s.rotation);
      roll = r.roll;
      pitch = r.pitch;
      planarity = r.planarity;
      if (src === 'fusion' && s.gyro && dt > 0) {
        // blend the gyro rate about the screen normal with the gravity angle
        roll = this.rollComp.filter(roll, -s.gyro.z, dt);
        this.fusion.update(s.gyro, s.gravity, dt);
      }
    }
    roll = this.rollFilter.filter(roll, dt);
    pitch = this.pitchFilter.filter(pitch, dt);
    this.tilt.roll = roll;
    this.tilt.pitch = pitch;
    this.tilt.planarity = planarity;
    // tilting the phone clockwise (right side down) steers to starboard
    const target = shapeAxis(roll, this.settings.steering);
    this.helm = this.settings.helmRate > 0 && dt > 0 ? rateLimit(this.helm, target, this.settings.helmRate, dt) : target;
    let throttle = 0;
    const throttleActive = this.settings.throttleMode === 'pitch';
    if (throttleActive) {
      // tilting the top edge away from you (like pushing a throttle lever) is ahead
      const th = shapeAxis(-pitch, this.settings.throttle);
      throttle = this.settings.throttle.allowAstern ? th : Math.max(0, th);
    }
    return { helm: this.helm, throttle, throttleActive };
  }
}

export interface WheelState {
  /** Wheel angle (rad), positive clockwise. */
  angle: number;
}

/**
 * On-screen helm wheel: convert a drag around the wheel centre into a wheel angle. `lock` is the
 * wheel angle for full rudder (e.g. 1.5 turns lock to lock = 3π/2 each way… typically ±π).
 */
export function dragWheel(state: WheelState, center: { x: number; y: number }, from: { x: number; y: number }, to: { x: number; y: number }, lock: number): WheelState {
  const a0 = Math.atan2(from.y - center.y, from.x - center.x);
  const a1 = Math.atan2(to.y - center.y, to.x - center.x);
  let d = a1 - a0;
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  // screen y grows downward, so a clockwise drag has positive d
  return { angle: clamp(state.angle + d, -lock, lock) };
}

export const wheelToHelm = (w: WheelState, lock: number): number => clamp(w.angle / lock, -1, 1);

export interface GamepadState {
  axes: readonly number[];
  buttons: readonly number[];
}

export interface GamepadMapping {
  steerAxis: number;
  /** Trigger buttons for ahead and astern (analogue 0…1), or an axis for throttle. */
  aheadButton: number;
  asternButton: number;
  deadzone: number;
  expo: number;
}

export const STANDARD_GAMEPAD: GamepadMapping = { steerAxis: 0, aheadButton: 7, asternButton: 6, deadzone: 0.12, expo: 0.3 };

/** Map a W3C "standard" gamepad to a helm command with a rescaled dead zone and expo. */
export function mapGamepad(g: GamepadState, m: GamepadMapping = STANDARD_GAMEPAD): HelmCommand {
  const shape = (v: number): number => {
    const a = Math.abs(v);
    if (a <= m.deadzone) return 0;
    const t = (a - m.deadzone) / (1 - m.deadzone);
    return Math.sign(v) * ((1 - m.expo) * t + m.expo * t * t * t);
  };
  const helm = shape(g.axes[m.steerAxis] ?? 0);
  const throttle = clamp((g.buttons[m.aheadButton] ?? 0) - (g.buttons[m.asternButton] ?? 0), -1, 1);
  return { helm, throttle };
}

export type InputSourceId = 'motion' | 'touch' | 'gamepad' | 'keyboard' | 'autopilot';

/**
 * Arbitrate several input sources per channel: the most recently *moved* source wins; a source that
 * has been idle for `idleTimeout` seconds yields to the default (lowest-priority active) one.
 */
export class InputMixer {
  private readonly last = new Map<string, { value: number; movedAt: number; source: InputSourceId }>();
  private readonly owner: Record<'helm' | 'throttle', InputSourceId | undefined> = { helm: undefined, throttle: undefined };
  constructor(
    public preferred: InputSourceId = 'motion',
    public idleTimeout = 3,
    public moveThreshold = 0.04,
  ) {}

  offer(source: InputSourceId, channel: 'helm' | 'throttle', value: number, t: number): void {
    const key = `${source}:${channel}`;
    const prev = this.last.get(key);
    const moved = !prev || Math.abs(value - prev.value) > this.moveThreshold;
    this.last.set(key, { value, movedAt: moved ? t : (prev?.movedAt ?? t), source });
    if (moved && prev) this.owner[channel] = source;
  }

  read(channel: 'helm' | 'throttle', t: number): { value: number; source: InputSourceId | undefined } {
    const own = this.owner[channel];
    if (own) {
      const e = this.last.get(`${own}:${channel}`);
      if (e && (t - e.movedAt < this.idleTimeout || own === this.preferred)) return { value: e.value, source: own };
    }
    const pref = this.last.get(`${this.preferred}:${channel}`);
    if (pref) return { value: pref.value, source: this.preferred };
    for (const e of this.last.values()) if (this.last.get(`${e.source}:${channel}`) === e) return { value: e.value, source: e.source };
    return { value: 0, source: undefined };
  }
}
