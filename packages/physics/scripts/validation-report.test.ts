/**
 * Generates docs/validation.md and the SVG plots in docs/img from the same runs the tests check.
 * Run with: REPORT=1 pnpm exec vitest run packages/physics/scripts --project physics-report
 * (or `pnpm validation-report` from the repo root).
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, it } from 'vitest';
import { calmWater } from '../src/core/environment';
import { presetEnvironment } from '@nautisim/environment';
import { createKVLCC2 } from '../src/hulls/kvlcc2';
import { createPlaningBoat, RIB_65, RUNABOUT_8 } from '../src/hulls/planing';
import { Simulation } from '../src/sim/simulation';
import { DEG_IN_RAD, KNOT_IN_MPS } from '../src/units';
import { type TrackPoint, turningCircle, zigzag } from '../src/validation/maneuvers';
import { KVLCC2_L7_REFERENCE as REF } from '../src/validation/references';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '../../..');

function svgPlot(opts: {
  title: string;
  series: Array<{ name: string; color: string; pts: Array<[number, number]> }>;
  xLabel: string;
  yLabel: string;
  equal?: boolean;
  hLines?: number[];
}): string {
  const W = 640;
  const H = 420;
  const m = { l: 56, r: 16, t: 34, b: 44 };
  const all = opts.series.flatMap((s) => s.pts);
  let x0 = Math.min(...all.map((p) => p[0]));
  let x1 = Math.max(...all.map((p) => p[0]));
  let y0 = Math.min(...all.map((p) => p[1]), ...(opts.hLines ?? []));
  let y1 = Math.max(...all.map((p) => p[1]), ...(opts.hLines ?? []));
  const pad = (a: number, b: number): [number, number] => {
    const d = (b - a) * 0.06 || 1;
    return [a - d, b + d];
  };
  [x0, x1] = pad(x0, x1);
  [y0, y1] = pad(y0, y1);
  const pw = W - m.l - m.r;
  const ph = H - m.t - m.b;
  if (opts.equal) {
    const sx = pw / (x1 - x0);
    const sy = ph / (y1 - y0);
    if (sx < sy) {
      const c = (y0 + y1) / 2;
      const half = ph / sx / 2;
      y0 = c - half;
      y1 = c + half;
    } else {
      const c = (x0 + x1) / 2;
      const half = pw / sy / 2;
      x0 = c - half;
      x1 = c + half;
    }
  }
  const X = (x: number): number => m.l + ((x - x0) / (x1 - x0)) * pw;
  const Y = (y: number): number => m.t + ph - ((y - y0) / (y1 - y0)) * ph;
  const ticks = (a: number, b: number): number[] => {
    const span = b - a;
    const step = 10 ** Math.floor(Math.log10(span / 5));
    const s = [1, 2, 5, 10].map((k) => k * step).find((k) => span / k <= 8) ?? step * 10;
    const out: number[] = [];
    for (let v = Math.ceil(a / s) * s; v <= b; v += s) out.push(Number(v.toFixed(6)));
    return out;
  };
  const parts: string[] = [];
  parts.push(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" font-family="system-ui, sans-serif" font-size="12" role="img">`,
  );
  parts.push(`<title>${opts.title}</title>`);
  parts.push(`<rect width="${W}" height="${H}" fill="#06121f"/>`);
  parts.push(`<text x="${W / 2}" y="20" fill="#e6f0f7" font-size="15" text-anchor="middle">${opts.title}</text>`);
  for (const t of ticks(x0, x1)) {
    parts.push(`<line x1="${X(t)}" y1="${m.t}" x2="${X(t)}" y2="${m.t + ph}" stroke="#16324a"/>`);
    parts.push(`<text x="${X(t)}" y="${H - m.b + 16}" fill="#8aa4b8" text-anchor="middle">${t}</text>`);
  }
  for (const t of ticks(y0, y1)) {
    parts.push(`<line x1="${m.l}" y1="${Y(t)}" x2="${m.l + pw}" y2="${Y(t)}" stroke="#16324a"/>`);
    parts.push(`<text x="${m.l - 6}" y="${Y(t) + 4}" fill="#8aa4b8" text-anchor="end">${t}</text>`);
  }
  for (const h of opts.hLines ?? []) parts.push(`<line x1="${m.l}" y1="${Y(h)}" x2="${m.l + pw}" y2="${Y(h)}" stroke="#8aa4b8" stroke-dasharray="4 4"/>`);
  parts.push(`<text x="${m.l + pw / 2}" y="${H - 8}" fill="#e6f0f7" text-anchor="middle">${opts.xLabel}</text>`);
  parts.push(`<text x="14" y="${m.t + ph / 2}" fill="#e6f0f7" text-anchor="middle" transform="rotate(-90 14 ${m.t + ph / 2})">${opts.yLabel}</text>`);
  opts.series.forEach((s, i) => {
    const d = s.pts.map((p, j) => `${j ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('');
    parts.push(`<path d="${d}" fill="none" stroke="${s.color}" stroke-width="2"/>`);
    parts.push(`<rect x="${m.l + 10}" y="${m.t + 8 + i * 18}" width="14" height="4" fill="${s.color}"/>`);
    parts.push(`<text x="${m.l + 30}" y="${m.t + 13 + i * 18}" fill="#e6f0f7">${s.name}</text>`);
  });
  parts.push('</svg>');
  return parts.join('\n');
}

describe.runIf(process.env.REPORT)('validation report', () => {
  it('writes docs/validation.md and plots', () => {
    mkdirSync(resolve(root, 'docs/img'), { recursive: true });
    const v = createKVLCC2({ scale: 45.714, dof: 'planar' });
    const sim = new Simulation(v, calmWater({ rho: 1025, nu: 1.0e-6, g: 9.81 }), { fixedStep: 0.05 });
    const L = v.dimensions.length;
    const start = { speed: REF.approachSpeed, rps: REF.selfPropulsionRps };
    const stb = turningCircle(sim, start, 35 * DEG_IN_RAD);
    const prt = turningCircle(sim, start, -35 * DEG_IN_RAD);
    const toXY = (tr: TrackPoint[]): Array<[number, number]> => tr.map((p) => [p.y / L, p.x / L]);
    writeFileSync(
      resolve(root, 'docs/img/kvlcc2-turning.svg'),
      svgPlot({
        title: 'KVLCC2 L7, 35° turning circles (NautiSim MMG)',
        series: [
          { name: '+35° (starboard)', color: '#3fbf5f', pts: toXY(stb.track) },
          { name: '−35° (port)', color: '#e04848', pts: toXY(prt.track) },
        ],
        xLabel: 'transfer y / Lpp',
        yLabel: 'advance x / Lpp',
        equal: true,
      }),
    );
    const zzKeys = [
      ['10/10', 10, 10],
      ['-10/-10', -10, 10],
      ['20/20', 20, 20],
      ['-20/-20', -20, 20],
    ] as const;
    const zz = zzKeys.map(([k, d, c]) => ({ k, r: zigzag(sim, start, d * DEG_IN_RAD, c * DEG_IN_RAD) }));
    const z1010 = zz[0]?.r;
    if (z1010) {
      writeFileSync(
        resolve(root, 'docs/img/kvlcc2-zigzag.svg'),
        svgPlot({
          title: 'KVLCC2 L7, 10°/10° zig-zag (NautiSim MMG)',
          series: [
            { name: 'heading ψ (°)', color: '#4fd1c5', pts: z1010.track.map((p) => [p.t, p.heading / DEG_IN_RAD]) },
            { name: 'rudder δ (°)', color: '#d6b25e', pts: z1010.track.map((p) => [p.t, p.rudder / DEG_IN_RAD]) },
          ],
          xLabel: 'time (s, model scale)',
          yLabel: 'angle (°)',
          hLines: [10, -10],
        }),
      );
    }
    // planing boats: speed and trim versus throttle
    const curves = [RIB_65, RUNABOUT_8].map((def) => {
      const b = createPlaningBoat(def);
      const rows: Array<{ throttle: number; kn: number; trim: number; rpm: number; lift: number }> = [];
      for (let th = 0.1; th <= 1.0001; th += 0.1) {
        const s = new Simulation(b, calmWater());
        s.controls = { throttle: th, helm: 0, thruster: 0 };
        s.run(Math.round(60 / s.dt));
        const sn = s.snapshot();
        rows.push({
          throttle: th,
          kn: sn.speedThroughWater / KNOT_IN_MPS,
          trim: sn.euler.pitch / DEG_IN_RAD,
          rpm: sn.engineRpm,
          lift: (sn.diag['planing.lift'] ?? 0) / (b.mass * 9.81),
        });
      }
      return { def, rows };
    });
    writeFileSync(
      resolve(root, 'docs/img/planing-speed.svg'),
      svgPlot({
        title: 'Planing boats: steady speed vs throttle (calm water)',
        series: curves.map((c, i) => ({ name: c.def.name, color: i ? '#d6b25e' : '#4fd1c5', pts: c.rows.map((r) => [r.throttle * 100, r.kn]) })),
        xLabel: 'throttle (%)',
        yLabel: 'speed through water (kn)',
      }),
    );
    const f = (x: number, d = 2): string => x.toFixed(d);
    const md: string[] = [];
    md.push('# Validation report');
    md.push('');
    md.push(
      'Generated by `pnpm validation-report` from the same simulations the test suite checks (`packages/physics/test`). Sources for the reference numbers are in `packages/physics/src/validation/references.ts` and [research.md](research.md).',
    );
    md.push('');
    md.push('## KVLCC2 manoeuvring (MMG, L7 model, Fn 0.142)');
    md.push('');
    md.push(
      `Propeller held at the model self-propulsion point, ${f(REF.selfPropulsionRps)} rps; rudder rate 15.8 °/s; fixed step 0.05 s, RK4, heave/roll/pitch locked (a 6-DOF run gives the same turning circle within 2 %).`,
    );
    md.push('');
    md.push('| Manoeuvre | Quantity | NautiSim | Yasukawa & Yoshimura MMG | MARIN free-running |');
    md.push('| --- | --- | --- | --- | --- |');
    md.push(`| +35° turn | advance / Lpp | ${f(stb.advance / L)} | ${REF.turning.starboard.mmg.advance} | ${REF.turning.starboard.marin.advance} |`);
    md.push(
      `| +35° turn | tactical diameter / Lpp | ${f(stb.tacticalDiameter / L)} | ${REF.turning.starboard.mmg.tacticalDiameter} | ${REF.turning.starboard.marin.tacticalDiameter} |`,
    );
    md.push(`| −35° turn | advance / Lpp | ${f(prt.advance / L)} | ${REF.turning.port.mmg.advance} | ${REF.turning.port.marin.advance} |`);
    md.push(
      `| −35° turn | tactical diameter / Lpp | ${f(prt.tacticalDiameter / L)} | ${REF.turning.port.mmg.tacticalDiameter} | ${REF.turning.port.marin.tacticalDiameter} |`,
    );
    for (const { k, r } of zz) {
      const ref = REF.zigzag[k];
      md.push(`| ${k} zig-zag | 1st overshoot (°) | ${f(r.firstOvershoot / DEG_IN_RAD, 1)} | ${ref.mmg[0]} | ${ref.marin[0]} |`);
      md.push(`| ${k} zig-zag | 2nd overshoot (°) | ${f(r.secondOvershoot / DEG_IN_RAD, 1)} | ${ref.mmg[1]} | ${ref.marin[1]} |`);
    }
    md.push('');
    md.push('![KVLCC2 turning circles](img/kvlcc2-turning.svg)');
    md.push('');
    md.push('![KVLCC2 10/10 zig-zag](img/kvlcc2-zigzag.svg)');
    md.push('');
    md.push(
      "Like Yasukawa & Yoshimura's own computation, the overshoot angles come out smaller than the free-running tests; the paper attributes this to the hull damping from the captive tests.",
    );
    md.push('');
    md.push('## Planing boats (Savitsky, time domain)');
    md.push('');
    md.push('Steady state after 60 s at each throttle setting in calm water.');
    md.push('');
    for (const c of curves) {
      md.push(`### ${c.def.name}`);
      md.push('');
      md.push('| Throttle | Speed (kn) | Trim (°) | Engine rpm | Dynamic lift / weight |');
      md.push('| --- | --- | --- | --- | --- |');
      for (const r of c.rows) md.push(`| ${Math.round(r.throttle * 100)} % | ${f(r.kn, 1)} | ${f(r.trim, 1)} | ${Math.round(r.rpm)} | ${f(r.lift)} |`);
      md.push('');
    }
    md.push('![Planing speed curves](img/planing-speed.svg)');
    md.push('');
    md.push(
      'The static Savitsky equations themselves are checked against a published worked example in `test/savitsky.test.ts`; hydrostatics against closed-form box-barge values in `test/hydrostatics.test.ts`.',
    );
    md.push('');
    writeFileSync(resolve(root, 'docs/validation.md'), md.join('\n'));
  }, 300_000);
});
