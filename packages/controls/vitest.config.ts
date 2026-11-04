import { defineProject } from 'vitest/config';

export default defineProject({
  test: {
    name: 'controls',
    include: ['test/**/*.test.ts'],
  },
});
