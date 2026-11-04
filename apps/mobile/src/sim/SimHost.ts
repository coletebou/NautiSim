import { Environment, SEA_STATES, type SeaStateId } from '@nautisim/environment';
import { findHull, type HullCatalogEntry, quatSlerp, Simulation, type Vessel, type VesselSnapshot } from '@nautisim/physics';

/** Wave components shared by physics and the ocean renderer, so both see the same surface. */
export const WAVE_COMPONENTS = 16;

/** Pose for one rendered frame, interpolated between the last two physics steps. */
export interface RenderPose {
  position: { x: number; y: number; z: number };
  orientation: { w: number; x: number; y: number; z: number };
  time: number;
}

export interface HostOptions {
  hullId: string;
  seaState: SeaStateId;
  waveFromDeg: number;
  seed?: number;
}

/**
 * Owns the simulation and its fixed-step clock (Fiedler's accumulator). Runs on the React Native
 * JS thread, which is separate from the platform UI thread; React never re-renders per frame —
 * the HUD pulls `latest` at a low rate through a tiny subscription.
 */
export class SimHost {
  readonly hull: HullCatalogEntry;
  readonly vessel: Vessel;
  readonly env: Environment;
  readonly sim: Simulation;
  private accumulator = 0;
  private prev: VesselSnapshot;
  latest: VesselSnapshot;
  paused = false;
  /** Physics milliseconds spent in the last frame (for the debug overlay). */
  lastStepMs = 0;
  private readonly listeners = new Set<() => void>();
  private lastEmit = -1;
  /** HUD refresh interval (s). */
  hudInterval = 0.1;
  static readonly MAX_SUBSTEPS = 8;

  constructor(o: HostOptions) {
    this.hull = findHull(o.hullId);
    this.vessel = this.hull.build({ resolution: 'physics' });
    const preset = SEA_STATES[o.seaState];
    const from = (o.waveFromDeg * Math.PI) / 180;
    this.env = new Environment({
      seed: o.seed ?? 7,
      sea: { hs: preset.hs, tp: preset.tp, fromDirection: from, spreading: 4 },
      wind: { speed: preset.wind, fromDirection: from, turbulence: 0.12 },
      waveComponents: WAVE_COMPONENTS,
    });
    this.sim = new Simulation(this.vessel, this.env);
    this.prev = this.sim.snapshot();
    this.latest = this.prev;
  }

  setControls(helm: number, throttle: number, thruster = 0): void {
    this.sim.controls = { helm, throttle, thruster };
  }

  /** Advance by real elapsed time; returns the interpolation factor for rendering. */
  advance(dtReal: number): number {
    if (this.paused) return 1;
    const dt = this.sim.dt;
    this.accumulator += Math.min(Math.max(dtReal, 0), 0.25);
    let n = 0;
    const t0 = Date.now();
    while (this.accumulator >= dt && n < SimHost.MAX_SUBSTEPS) {
      this.prev = this.latest;
      this.sim.step();
      this.latest = this.sim.snapshot();
      this.accumulator -= dt;
      n++;
    }
    // if the device cannot keep up, drop time rather than spiral
    if (n === SimHost.MAX_SUBSTEPS) this.accumulator = 0;
    this.lastStepMs = Date.now() - t0;
    if (this.latest.t - this.lastEmit >= this.hudInterval) {
      this.lastEmit = this.latest.t;
      for (const l of this.listeners) l();
    }
    return this.accumulator / dt;
  }

  pose(alpha: number): RenderPose {
    const a = this.prev;
    const b = this.latest;
    const k = Math.min(Math.max(alpha, 0), 1);
    return {
      position: {
        x: a.position.x + (b.position.x - a.position.x) * k,
        y: a.position.y + (b.position.y - a.position.y) * k,
        z: a.position.z + (b.position.z - a.position.z) * k,
      },
      orientation: quatSlerp(a.orientation, b.orientation, k),
      time: a.t + (b.t - a.t) * k,
    };
  }

  subscribe(l: () => void): () => void {
    this.listeners.add(l);
    return () => this.listeners.delete(l);
  }

  getSnapshot = (): VesselSnapshot => this.latest;

  reset(): void {
    this.sim.reset();
    this.accumulator = 0;
    this.prev = this.sim.snapshot();
    this.latest = this.prev;
  }
}
