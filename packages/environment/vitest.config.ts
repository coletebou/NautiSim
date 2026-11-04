import { defineProject } from 'vitest/config';

export default defineProject({
  test: {
    name: 'environment',
    include: ['test/**/*.test.ts'],
  },
});
