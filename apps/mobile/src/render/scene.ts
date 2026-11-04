import type { WaveField } from '@nautisim/environment';
import type { Vessel, VesselSnapshot } from '@nautisim/physics';
import * as THREE from 'three/webgpu';
import type { RenderPose } from '../sim/SimHost';
import type { CameraMode, Quality } from '../state/settings';
import { nedToThree, quatNedToThree } from './coords';
import { OceanView } from './ocean';
import { buildVesselModel } from './vesselModel';
import { WakeView } from './wake';

const QUALITY = {
  low: { grid: 48, sizeFactor: 1 },
  medium: { grid: 72, sizeFactor: 1 },
  high: { grid: 104, sizeFactor: 1.2 },
} as const;

export interface SceneParams {
  vessel: Vessel;
  displayVessel: Vessel;
  waves: WaveField;
  quality: Quality;
}

/**
 * Everything drawn in 3D, independent of the GPU backend: sky and fog, lights, ocean, vessel, wake
 * and cameras. `frame()` positions it all from an interpolated physics pose.
 */
export class NauticalScene {
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  private readonly ocean: OceanView;
  private readonly vesselObj: THREE.Group;
  private readonly wake: WakeView;
  private readonly L: number;
  private readonly chase = { yaw: 0, init: false, pos: new THREE.Vector3() };
  /** Orbit camera angles, driven by touch drags. */
  orbit = { yaw: 0.6, pitch: 0.35, distance: 1 };
  private readonly tmp = { x: 0, y: 0, z: 0 };
  private lastT = 0;

  constructor(private readonly p: SceneParams) {
    const L = p.vessel.dimensions.length;
    this.L = L;
    const far = Math.max(4000, 40 * L);
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.1, far);
    this.scene.background = new THREE.Color(0x9cc3dc);
    this.scene.fog = new THREE.Fog(0x9cc3dc, Math.max(300, 6 * L), far * 0.9);
    this.scene.add(new THREE.HemisphereLight(0xdfefff, 0x0a2233, 1.6));
    const sun = new THREE.DirectionalLight(0xfff2dc, 2.2);
    sun.position.set(-300, 400, -200);
    this.scene.add(sun);
    const q = QUALITY[p.quality];
    const size = Math.max(160, Math.min(2400, 12 * L)) * q.sizeFactor;
    this.ocean = new OceanView(p.waves, { resolution: q.grid, size });
    this.scene.add(this.ocean.group);
    this.vesselObj = buildVesselModel(p.displayVessel);
    this.scene.add(this.vesselObj);
    this.wake = new WakeView(p.vessel.dimensions.beam, Math.max(1.5, L / 25));
    this.scene.add(this.wake.mesh);
  }

  resize(width: number, height: number): void {
    this.camera.aspect = width / Math.max(height, 1);
    this.camera.updateProjectionMatrix();
  }

  frame(pose: RenderPose, snap: VesselSnapshot, mode: CameraMode): void {
    const dt = Math.min(Math.max(pose.time - this.lastT, 0), 0.1);
    this.lastT = pose.time;
    const pos = nedToThree(pose.position, this.tmp);
    this.vesselObj.position.set(pos.x, pos.y, pos.z);
    const q = quatNedToThree(pose.orientation);
    this.vesselObj.quaternion.set(q.x, q.y, q.z, q.w);
    this.ocean.update(pos.x, pos.z, pose.time);
    // wake from the stern, along the direction of travel
    const heading = snap.heading;
    const fwdX = Math.sin(heading);
    const fwdZ = -Math.cos(heading);
    const sternDist = 0.45 * this.L;
    const waves = this.p.waves;
    this.wake.update(pos.x - fwdX * sternDist, pos.z - fwdZ * sternDist, fwdX, fwdZ, snap.speedThroughWater, dt, (x, z) => waves.elevation(-z, x, pose.time));
    this.placeCamera(pos, heading, mode, dt);
  }

  private placeCamera(pos: { x: number; y: number; z: number }, heading: number, mode: CameraMode, dt: number): void {
    const L = this.L;
    const cam = this.camera;
    if (mode === 'bridge') {
      const eye = this.p.vessel.bridgeEye;
      const e = new THREE.Vector3(eye.y, -eye.z, -eye.x).applyQuaternion(this.vesselObj.quaternion).add(this.vesselObj.position);
      cam.position.copy(e);
      // look ahead along the bow, levelled partly so the horizon does not swing with every roll
      const ahead = new THREE.Vector3(0, -0.05, -1).applyQuaternion(this.vesselObj.quaternion);
      ahead.y *= 0.4;
      cam.lookAt(e.clone().add(ahead.multiplyScalar(100)));
      cam.fov = 65;
    } else if (mode === 'orbit') {
      const d = this.orbit.distance * Math.max(14, 1.8 * L);
      const yaw = heading + this.orbit.yaw;
      cam.position.set(
        pos.x - Math.sin(yaw) * Math.cos(this.orbit.pitch) * d,
        pos.y + Math.sin(this.orbit.pitch) * d,
        pos.z + Math.cos(yaw) * Math.cos(this.orbit.pitch) * d,
      );
      cam.lookAt(pos.x, pos.y, pos.z);
      cam.fov = 55;
    } else {
      // chase: trails behind on a lagged heading so turns read clearly
      if (!this.chase.init) {
        this.chase.yaw = heading;
        this.chase.init = true;
      }
      const k = 1 - Math.exp(-dt * 1.5);
      let dy = heading - this.chase.yaw;
      dy = Math.atan2(Math.sin(dy), Math.cos(dy));
      this.chase.yaw += dy * k;
      const back = Math.max(12, 2.2 * L);
      const up = Math.max(4, 0.55 * L);
      const target = new THREE.Vector3(pos.x - Math.sin(this.chase.yaw) * back, Math.max(pos.y, 0) + up, pos.z + Math.cos(this.chase.yaw) * back);
      if (this.chase.pos.lengthSq() === 0) this.chase.pos.copy(target);
      this.chase.pos.lerp(target, 1 - Math.exp(-dt * 4));
      cam.position.copy(this.chase.pos);
      cam.lookAt(pos.x + Math.sin(heading) * 0.3 * L, pos.y + 0.1 * L, pos.z - Math.cos(heading) * 0.3 * L);
      cam.fov = 60;
    }
    cam.updateProjectionMatrix();
  }

  dispose(): void {
    this.ocean.dispose();
    this.wake.dispose();
    this.scene.traverse((o) => {
      const m = o as THREE.Mesh;
      if (m.geometry) m.geometry.dispose();
    });
  }
}
