import { defineProject } from 'vitest/config';

export default defineProject({
  test: {
    name: 'physics',
    include: ['test/**/*.test.ts', 'scripts/**/*.test.ts'],
  },
});
