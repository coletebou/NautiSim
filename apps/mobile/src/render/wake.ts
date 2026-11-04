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
    const mat = new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.3, depthWrite: false, side: THREE.DoubleSide });
    this.mesh = new THREE.Mesh(this.geometry, mat);
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 2;
  }

  /** Stern position and heading direction (three.js x/z), boat speed (m/s), frame dt (s). */
  update(x: number, z: number, dirX: number, dirZ: number, speed: number, dt: number, heightAt: (x: number, z: number) => number): void {
    for (const p of this.pts) p.age += dt;
    // pts[0] is a head that rides with the stern; a new point is committed once the head is
    // `spacing` metres past the last committed one
    const head = this.pts[0];
    const prev = this.pts[1];
    if (!head || !prev || Math.hypot(x - prev.x, z - prev.z) > this.spacing) {
      this.pts.unshift({ x, z, dx: dirX, dz: dirZ, age: 0, speed });
      if (this.pts.length > this.maxPoints) this.pts.pop();
    } else {
      head.x = x;
      head.z = z;
      head.dx = dirX;
      head.dz = dirZ;
      head.age = 0;
      head.speed = speed;
    }
    const P = this.positions;
    const C = this.colors;
    for (let i = 0; i < this.maxPoints; i++) {
      const p = this.pts[Math.min(i, this.pts.length - 1)];
      if (!p) break;
      // the ribbon follows the path actually travelled, so use the direction between points
      const q = this.pts[Math.min(i + 1, this.pts.length - 1)] ?? p;
      let dx = p.x - q.x;
      let dz = p.z - q.z;
      const len = Math.hypot(dx, dz);
      if (len > 1e-3) {
        dx /= len;
        dz /= len;
      } else {
        dx = p.dx;
        dz = p.dz;
      }
      const width = Math.min(this.beam * (0.6 + 0.012 * p.age * Math.min(p.speed, 10)), this.beam * 1.6);
      const nx = -dz;
      const nz = dx;
      const fresh = Math.max(0, 1 - p.age / 30) * Math.min(1, p.speed / 3);
      const y = heightAt(p.x, p.z) + 0.04;
      for (let s = 0; s < 2; s++) {
        const sign = s === 0 ? -1 : 1;
        const o = (2 * i + s) * 3;
        P[o] = p.x + nx * sign * width * 0.5;
        P[o + 1] = y;
        P[o + 2] = p.z + nz * sign * width * 0.5;
        // blend from the water colour to foam white while the wake is fresh
        const f = fresh * fresh;
        const c = (2 * i + s) * 3;
        C[c] = 0.043 + 0.88 * f;
        C[c + 1] = 0.24 + 0.72 * f;
        C[c + 2] = 0.36 + 0.62 * f;
      }
    }
    (this.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    (this.geometry.getAttribute('color') as THREE.BufferAttribute).needsUpdate = true;
  }

  dispose(): void {
    this.geometry.dispose();
  }
}
