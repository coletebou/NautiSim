import type { WaveField } from '@nautisim/environment';
import * as THREE from 'three/webgpu';

export interface OceanOptions {
  /** Vertices per side of the near grid. */
  resolution: number;
  /** Side length of the near grid (m). */
  size: number;
}

/**
 * Ocean surface. A grid that follows the camera (snapped to its own spacing so it does not swim)
 * is displaced on the CPU every frame by the same wave field the physics uses, so the hull floats
 * on exactly the waves on screen. Beyond it a large flat skirt fades into the fog.
 */
export class OceanView {
  readonly group = new THREE.Group();
  private readonly geometry: THREE.BufferGeometry;
  private readonly positions: Float32Array;
  private readonly normals: Float32Array;
  private readonly heights: Float32Array;
  private readonly n: number;
  private readonly spacing: number;
  private readonly comps: Float32Array;
  private readonly count: number;

  constructor(waves: WaveField, o: OceanOptions) {
    const n = o.resolution;
    this.n = n;
    this.spacing = o.size / (n - 1);
    const r = waves.renderComponents();
    this.comps = r.data;
    this.count = r.count;
    this.positions = new Float32Array(n * n * 3);
    this.normals = new Float32Array(n * n * 3);
    this.heights = new Float32Array(n * n);
    const index: number[] = [];
    for (let j = 0; j < n - 1; j++) {
      for (let i = 0; i < n - 1; i++) {
        const a = j * n + i;
        const b = a + 1;
        const c = a + n;
        const d = c + 1;
        index.push(a, c, b, b, c, d);
      }
    }
    this.geometry = new THREE.BufferGeometry();
    this.geometry.setIndex(index);
    this.geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
    this.geometry.setAttribute('normal', new THREE.BufferAttribute(this.normals, 3));
    const water = new THREE.MeshStandardMaterial({ color: 0x0b3d5c, roughness: 0.18, metalness: 0.05 });
    const near = new THREE.Mesh(this.geometry, water);
    near.frustumCulled = false;
    this.group.add(near);
    // flat ring from the grid's inscribed circle out to the horizon
    const skirt = new THREE.Mesh(new THREE.RingGeometry(o.size * 0.49, 20000, 64, 1), new THREE.MeshStandardMaterial({ color: 0x0b3d5c, roughness: 0.3 }));
    skirt.rotation.x = -Math.PI / 2;
    this.group.add(skirt);
    this.skirt = skirt;
  }

  private readonly skirt: THREE.Mesh;

  /** Rebuild the grid around (centreEast, centreSouth) in three.js coordinates at time t. */
  update(centreX: number, centreZ: number, t: number): void {
    const n = this.n;
    const h = this.spacing;
    const half = ((n - 1) * h) / 2;
    const x0 = Math.round(centreX / h) * h - half;
    const z0 = Math.round(centreZ / h) * h - half;
    const c = this.comps;
    const nc = this.count;
    const P = this.positions;
    const H = this.heights;
    for (let j = 0; j < n; j++) {
      const zt = z0 + j * h;
      for (let i = 0; i < n; i++) {
        const xt = x0 + i * h;
        // three (x east, z south) → NED (north = −z, east = x)
        const north = -zt;
        const east = xt;
        // fade the waves out over the outer quarter of the grid so its edge meets the flat skirt
        const ex = Math.min(i, n - 1 - i) / (n - 1);
        const ez = Math.min(j, n - 1 - j) / (n - 1);
        const edge = Math.min(ex, ez) / 0.12;
        const fade = edge >= 1 ? 1 : edge * edge * (3 - 2 * edge);
        let eta = 0;
        for (let k = 0; fade > 0 && k < nc; k++) {
          const o = k * 5;
          eta += (c[o] as number) * Math.cos((c[o + 1] as number) * north + (c[o + 2] as number) * east - (c[o + 3] as number) * t + (c[o + 4] as number));
        }
        eta *= fade;
        const idx = j * n + i;
        H[idx] = eta;
        P[idx * 3] = xt;
        P[idx * 3 + 1] = eta;
        P[idx * 3 + 2] = zt;
      }
    }
    // normals from central differences of the height grid
    const N = this.normals;
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const idx = j * n + i;
        const hl = H[j * n + Math.max(i - 1, 0)] as number;
        const hr = H[j * n + Math.min(i + 1, n - 1)] as number;
        const hd = H[Math.max(j - 1, 0) * n + i] as number;
        const hu = H[Math.min(j + 1, n - 1) * n + i] as number;
        const dx = (hr - hl) / (2 * h);
        const dz = (hu - hd) / (2 * h);
        const inv = 1 / Math.hypot(dx, 1, dz);
        N[idx * 3] = -dx * inv;
        N[idx * 3 + 1] = inv;
        N[idx * 3 + 2] = -dz * inv;
      }
    }
    const pos = this.geometry.getAttribute('position') as THREE.BufferAttribute;
    const nor = this.geometry.getAttribute('normal') as THREE.BufferAttribute;
    pos.needsUpdate = true;
    nor.needsUpdate = true;
    this.skirt.position.x = centreX;
    this.skirt.position.z = centreZ;
  }

  dispose(): void {
    this.geometry.dispose();
  }
}
