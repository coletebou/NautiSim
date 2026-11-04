import * as THREE from 'three/webgpu';

/**
 * Foam wake: a ribbon through recent stern positions, widening and fading with age. Points are
 * dropped every `spacing` metres travelled, so the ribbon is independent of frame rate.
 */
export class WakeView {
  readonly mesh: THREE.Mesh;
  private readonly maxPoints: number;
  private readonly pts: Array<{ x: number; z: number; dx: number; dz: number; age: number; speed: number }> = [];
  private readonly positions: Float32Array;
  private readonly colors: Float32Array;
  private readonly geometry = new THREE.BufferGeometry();

  constructor(
    private readonly beam: number,
    private readonly spacing: number,
    maxPoints = 120,
  ) {
    this.maxPoints = maxPoints;
    this.positions = new Float32Array(maxPoints * 2 * 3);
    this.colors = new Float32Array(maxPoints * 2 * 3);
    const idx: number[] = [];
    for (let i = 0; i < maxPoints - 1; i++) {
      const a = 2 * i;
      idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
    }
    this.geometry.setIndex(idx);
    this.geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
    const mat = new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.85, depthWrite: false, side: THREE.DoubleSide });
    this.mesh = new THREE.Mesh(this.geometry, mat);
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 2;
  }

  /** Stern position and heading direction (three.js x/z), boat speed (m/s), frame dt (s). */
  update(x: number, z: number, dirX: number, dirZ: number, speed: number, dt: number, heightAt: (x: number, z: number) => number): void {
    for (const p of this.pts) p.age += dt;
    const last = this.pts[0];
    if (!last || Math.hypot(x - last.x, z - last.z) > this.spacing) {
      this.pts.unshift({ x, z, dx: dirX, dz: dirZ, age: 0, speed });
      if (this.pts.length > this.maxPoints) this.pts.pop();
    } else {
      last.x = x;
      last.z = z;
    }
    const P = this.positions;
    const C = this.colors;
    for (let i = 0; i < this.maxPoints; i++) {
      const p = this.pts[Math.min(i, this.pts.length - 1)];
      if (!p) break;
      const width = this.beam * (0.5 + 0.12 * p.age * Math.min(p.speed, 12));
      // perpendicular to the direction of travel
      const nx = -p.dz;
      const nz = p.dx;
      const fade = Math.max(0, 1 - p.age / 40) * Math.min(1, p.speed / 2);
      const y = heightAt(p.x, p.z) + 0.05;
      for (let s = 0; s < 2; s++) {
        const sign = s === 0 ? -1 : 1;
        const o = (2 * i + s) * 3;
        P[o] = p.x + nx * sign * width * 0.5;
        P[o + 1] = y;
        P[o + 2] = p.z + nz * sign * width * 0.5;
        // blend from the water colour to foam white as the wake is fresh
        C[o] = 0.043 + 0.85 * fade;
        C[o + 1] = 0.24 + 0.68 * fade;
        C[o + 2] = 0.36 + 0.58 * fade;
      }
    }
    (this.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    (this.geometry.getAttribute('color') as THREE.BufferAttribute).needsUpdate = true;
  }

  dispose(): void {
    this.geometry.dispose();
  }
}
