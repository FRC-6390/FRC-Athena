import { describe, expect, test } from 'bun:test';
import { lineDiff } from './line-diff';

describe('lineDiff', () => {
  test('marks changed lines as removed then added', () => {
    const diff = lineDiff('kP=1\nkI=0\nkD=2', 'kP=1\nkI=0.5\nkD=2');
    expect(diff).toEqual([
      { kind: 'same', text: 'kP=1' },
      { kind: 'removed', text: 'kI=0' },
      { kind: 'added', text: 'kI=0.5' },
      { kind: 'same', text: 'kD=2' }
    ]);
  });

  test('identical text has no changes', () => {
    expect(lineDiff('a\nb', 'a\nb').every((line) => line.kind === 'same')).toBe(true);
  });
});
