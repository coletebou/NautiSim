/**
 * Browser harness that renders the app's real scene code (apps/mobile/src/render) with three.js
 * (WebGL2 backend) after running the real physics for a few seconds. Used to make README
 * screenshots headlessly: `pnpm screenshot`.
 */
import * as THREE from 'three/webgpu';
import { NauticalScene } from '../../apps/mobile/src/render/scene';
import { SimHost } from '../../apps/mobile/src/sim/SimHost';

const params = new URLSearchParams(location.search);
const hullId = params.get('hull') ?? 'rib';
const camera = (params.get('camera') ?? 'chase') as 'chase' | 'bridge' | 'orbit';
const sea = (params.get('sea') ?? 'moderate') as 'calm' | 'smooth' | 'slight' | 'moderate' | 'rough' | 'veryRough';
const seconds = Number(params.get('t') ?? '25');
const throttle = Number(params.get('throttle') ?? '0.75');
const helm = Number(params.get('helm') ?? '0.25');
const v0 = Number(params.get('v0') ?? '0');
const orbitDistance = Number(params.get('dist') ?? '0.9');

async function main(): Promise<void> {
  const canvas = document.getElementById('c') as HTMLCanvasElement;
  const host = new SimHost({ hullId, seaState: sea, waveFromDeg: 40 });
  const scene = new NauticalScene({ vessel: host.vessel, displayVessel: host.hull.build({ resolution: 'display' }), waves: host.env.waves, quality: 'high' });
  if (camera === 'orbit') scene.orbit = { yaw: 2.2, pitch: 0.28, distance: orbitDistance };
  if (v0 > 0) {
    host.sim.reset({ speed: v0 });
    host.sim.overrideActuators({ rps: throttle * host.vessel.actuators.rpsMax });
  }
  const renderer = new THREE.WebGPURenderer({ canvas, antialias: true, forceWebGL: true });
  renderer.setPixelRatio(1);
  renderer.setSize(window.innerWidth, window.innerHeight, false);
  await renderer.init();
  scene.resize(window.innerWidth, window.innerHeight);
  host.setControls(0, throttle);
  // run physics ahead in simulated time, rendering the scene so the wake and chase camera settle
  const dt = 1 / 30;
  for (let t = 0; t < seconds; t += dt) {
    if (t > seconds * 0.5) host.setControls(helm, throttle);
    const a = host.advance(dt);
    scene.frame(host.pose(a), host.latest, camera);
  }
  renderer.render(scene.scene, scene.camera);
  const s = host.latest;
  const kn = (s.speedThroughWater / 0.514444).toFixed(1);
  const hdg = (((s.heading * 180) / Math.PI + 360) % 360).toFixed(0).padStart(3, '0');
  (document.getElementById('hud') as HTMLDivElement).textContent =
    `${host.hull.name} · ${kn} kn · HDG ${hdg}° · heel ${((s.euler.roll * 180) / Math.PI).toFixed(1)}° · trim ${((s.euler.pitch * 180) / Math.PI).toFixed(1)}°`;
  const dbg = (scene as unknown as { wake: { positions: Float32Array; pts: unknown[] } }).wake;
  if (params.get('debug'))
    (document.getElementById('hud') as HTMLDivElement).textContent += ` | wake ${dbg.pts.length} ${Array.from(dbg.positions.slice(0, 12))
      .map((v) => v.toFixed(1))
      .join(',')}`;
  document.title = 'ready';
}

main().catch((e) => {
  document.body.textContent = String(e);
  document.title = 'error';
});
