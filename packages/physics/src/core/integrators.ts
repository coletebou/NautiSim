import { newWrench, type Wrench } from './types';
import { type BodyProperties, enforceConstraints, rigidBodyDerivative, STATE_SIZE, type WrenchFn } from './rigidbody';

export type IntegratorKind = 'rk4' | 'semi-implicit-euler';

export interface Integrator {
  readonly kind: IntegratorKind;
  /** Advance y in place from t to t + h. Returns the wrench of the first evaluation (for diagnostics). */
  step(y: Float64Array, t: number, h: number, body: Readonly<BodyProperties>, wrenchFn: WrenchFn): Readonly<Wrench>;
}

function zeroWrench(w: Wrench): void {
  w.force.x = 0;
  w.force.y = 0;
  w.force.z = 0;
  w.moment.x = 0;
  w.moment.y = 0;
  w.moment.z = 0;
}

/** Classic 4th-order Runge–Kutta on the full 13-state vector, quaternion renormalised after. */
export function createRK4(): Integrator {
  const k1 = new Float64Array(STATE_SIZE);
  const k2 = new Float64Array(STATE_SIZE);
  const k3 = new Float64Array(STATE_SIZE);
  const k4 = new Float64Array(STATE_SIZE);
  const tmp = new Float64Array(STATE_SIZE);
  const w1 = newWrench();
  const w = newWrench();
  const stage = (y: Float64Array, k: Float64Array, a: number, out: Float64Array): void => {
    for (let i = 0; i < STATE_SIZE; i++) out[i] = (y[i] as number) + a * (k[i] as number);
  };
  return {
    kind: 'rk4',
    step(y, t, h, body, wrenchFn) {
      zeroWrench(w1);
      wrenchFn(y, t, w1);
      rigidBodyDerivative(y, w1, body, k1);
      stage(y, k1, h / 2, tmp);
      zeroWrench(w);
      wrenchFn(tmp, t + h / 2, w);
      rigidBodyDerivative(tmp, w, body, k2);
      stage(y, k2, h / 2, tmp);
      zeroWrench(w);
      wrenchFn(tmp, t + h / 2, w);
      rigidBodyDerivative(tmp, w, body, k3);
      stage(y, k3, h, tmp);
      zeroWrench(w);
      wrenchFn(tmp, t + h, w);
      rigidBodyDerivative(tmp, w, body, k4);
      for (let i = 0; i < STATE_SIZE; i++) {
        y[i] = (y[i] as number) + (h / 6) * ((k1[i] as number) + 2 * (k2[i] as number) + 2 * (k3[i] as number) + (k4[i] as number));
      }
      enforceConstraints(y, body.mask);
      return w1;
    },
  };
}

/**
 * Semi-implicit (symplectic) Euler: velocities first from the current forces, then positions
 * and attitude from the new velocities. One force evaluation per step.
 */
export function createSemiImplicitEuler(): Integrator {
  const k = new Float64Array(STATE_SIZE);
  const w1 = newWrench();
  return {
    kind: 'semi-implicit-euler',
    step(y, t, h, body, wrenchFn) {
      zeroWrench(w1);
      wrenchFn(y, t, w1);
      rigidBodyDerivative(y, w1, body, k);
      for (let i = 7; i < STATE_SIZE; i++) y[i] = (y[i] as number) + h * (k[i] as number);
      enforceConstraints(y, body.mask);
      rigidBodyDerivative(y, w1, body, k);
      for (let i = 0; i < 7; i++) y[i] = (y[i] as number) + h * (k[i] as number);
      enforceConstraints(y, body.mask);
      return w1;
    },
  };
}

export function createIntegrator(kind: IntegratorKind): Integrator {
  return kind === 'rk4' ? createRK4() : createSemiImplicitEuler();
}
