import type { Vessel } from '@nautisim/physics';
import * as THREE from 'three/webgpu';
import { bodyPositionsToThree } from './coords';

/**
 * Visual model of a vessel. The hull is the physics hull mesh at display resolution (so what you
 * see is what floats), coloured by height: antifouling below the design waterline, topsides above.
 * Superstructure is simple primitives per hull type. Object space is three.js body space with the
 * origin at the centre of gravity.
 */
export function buildVesselModel(display: Vessel): THREE.Group {
  const group = new THREE.Group();
  const pos = bodyPositionsToThree(display.mesh.positions);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setIndex(Array.from(display.mesh.indices));
  geo.computeVertexNormals();
  const isShip = display.id.startsWith('kvlcc2');
  // waterline height in object space: the CG sits rest.z below the surface (NED), so the surface is
  // at three y = rest.z above the origin
  const wl = display.rest.z;
  const colors = new Float32Array(pos.length);
  const bottom = isShip ? new THREE.Color(0x8a1f17) : new THREE.Color(0x2b3640);
  const boot = new THREE.Color(0x101418);
  const top = isShip ? new THREE.Color(0x1c2833) : new THREE.Color(0xe8ecef);
  for (let i = 0; i < pos.length; i += 3) {
    const y = pos[i + 1] as number;
    const c = y < wl - 0.02 * display.dimensions.draft ? bottom : y < wl + 0.03 * display.dimensions.depth ? boot : top;
    colors[i] = c.r;
    colors[i + 1] = c.g;
    colors[i + 2] = c.b;
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  const hull = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.1, side: THREE.DoubleSide }));
  group.add(hull);
  const L = display.dimensions.length;
  const B = display.dimensions.beam;
  const D = display.dimensions.depth;
  const deckY = wl + (D - display.dimensions.draft);
  const mat = (c: number, rough = 0.6): THREE.MeshStandardMaterial => new THREE.MeshStandardMaterial({ color: c, roughness: rough });
  const box = (w: number, h: number, l: number, c: number, x: number, y: number, z: number): THREE.Mesh => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, l), mat(c));
    m.position.set(x, y, z);
    group.add(m);
    return m;
  };
  // the CG is xG forward of midship for the ship and lcg forward of the transom for boats; in object
  // space +z is aft. Bow is at z ≈ −(L/2 − xG) for the ship.
  if (isShip) {
    const xG = (display.meta.xG as number | undefined) ?? 0.035 * L;
    const aft = L / 2 + xG; // distance from the CG to the stern
    // deck
    box(B * 0.98, 0.4, L * 0.92, 0x5a6b58, 0, deckY + 0.2, 0.02 * L);
    // accommodation block and bridge near the stern
    box(B * 0.7, 22, L * 0.08, 0xe7e4dc, 0, deckY + 11, aft - 0.12 * L);
    box(B * 0.95, 3, L * 0.06, 0xe7e4dc, 0, deckY + 23.5, aft - 0.12 * L);
    // funnel
    box(B * 0.18, 12, L * 0.03, 0x24313c, 0, deckY + 28, aft - 0.07 * L);
    // manifold and pipework hint
    box(B * 0.6, 2, L * 0.03, 0x8a6d3b, 0, deckY + 1.4, -0.05 * L);
  } else {
    const isRib = display.id === 'rib';
    const lcg = (display.meta.lcg as number | undefined) ?? 0.37 * L;
    const sternZ = lcg; // transom is lcg aft of the CG
    if (isRib) {
      // inflatable tubes along both sides
      const tubeR = 0.25;
      for (const side of [-1, 1]) {
        const tube = new THREE.Mesh(new THREE.CylinderGeometry(tubeR, tubeR, L * 0.82, 16), mat(0x3c4247, 0.8));
        tube.rotation.x = Math.PI / 2;
        tube.position.set(side * (B / 2 - tubeR * 0.6), deckY - 0.05, sternZ - L * 0.43);
        group.add(tube);
      }
      // console with a small screen
      box(0.7, 1.0, 0.6, 0xdfe3e6, 0, deckY + 0.5, sternZ - 0.45 * L);
      box(0.6, 0.35, 0.05, 0x1b2733, 0, deckY + 1.15, sternZ - 0.45 * L - 0.28);
    } else {
      // bowrider: deck, windscreen and a bimini frame
      box(B * 0.9, 0.08, L * 0.85, 0xc9b99a, 0, deckY, sternZ - L * 0.45);
      const screen = box(B * 0.85, 0.45, 0.06, 0x2a3f52, 0, deckY + 0.3, sternZ - 0.55 * L);
      screen.rotation.x = -0.5;
    }
    // outboard / drive leg at the transom
    box(0.35, 0.9, 0.4, 0x15191d, 0, deckY - 0.1, sternZ + 0.25);
  }
  return group;
}
