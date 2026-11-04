# Contributing to NautiSim

Thanks for helping. A few ground rules keep the simulator trustworthy.

## Setup

```sh
corepack enable
pnpm install
pnpm check        # typecheck + Biome + tests
```

The mobile app lives in `apps/mobile` (`npx expo start`). The 3D view needs a development build (`npx expo run:ios|android`); Expo Go shows the 2-D chart.

## Where things go

- `packages/physics` — anything that computes forces or motion. Must stay free of React, React Native, DOM APIs, `Math.random` and wall-clock time. SI units inside; knots and degrees only at the edges (`src/units.ts`).
- `packages/environment` — waves, wind, current, depth. Seeded and deterministic.
- `packages/controls` — input processing. Pure functions or small explicit-state classes, no sensor APIs.
- `apps/mobile` — everything platform-specific.

## Physics changes need evidence

- New force models or coefficient changes come with a test, ideally against a published number (put the source and the values in `packages/physics/src/validation/references.ts`).
- If a value has no published source, say so in a comment ("calibrated", "estimate") and in the PR.
- Run `pnpm validation-report` and commit the updated `docs/validation.md` when results move.

## Style

- TypeScript strict. Biome formats and lints (`pnpm format`).
- Conventional Commits: `feat(physics): …`, `fix(mobile): …`, `docs: …`, `test(controls): …`, `chore: …`.
- Keep PRs focused; explain the why in the description.

## Secrets

Never commit credentials, signing keys or `.env` files. `eas.json` holds build profiles only; EAS credentials live in your Expo account. A gitleaks hook scans pushes.
