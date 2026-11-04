import { resolve } from 'node:path';
import { defineConfig } from 'vite';

const root = resolve(__dirname, '../..');
export default defineConfig({
  root: __dirname,
  base: './',
  resolve: {
    alias: {
      '@nautisim/physics': resolve(root, 'packages/physics/src/index.ts'),
      '@nautisim/environment': resolve(root, 'packages/environment/src/index.ts'),
      '@nautisim/controls': resolve(root, 'packages/controls/src/index.ts'),
    },
  },
  build: { outDir: resolve(root, 'tools/preview/dist'), emptyOutDir: true, target: 'es2022' },
});
