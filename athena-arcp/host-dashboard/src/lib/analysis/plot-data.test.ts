import { describe, expect, test } from 'bun:test';
import { decimate, toClipSpace, windowPoints, yRangeOf } from './plot-data';

describe('plot data', () => {
  test('the window starts with the sample in effect at its start', () => {
    const pts = windowPoints([0, 100, 200, 300], [1, 2, 3, 4], 150, 300, 0);
    expect(pts).toEqual([
      { x: 0.0001, y: 2 },
      { x: 0.0002, y: 3 },
      { x: 0.0003, y: 4 }
    ]);
  });

  test('samples after the window end are excluded', () => {
    const pts = windowPoints([0, 100, 200], [1, 2, 3], 0, 100, 0);
    expect(pts.map((p) => p.y)).toEqual([1, 2]);
  });

  test('decimation keeps about the requested number of points', () => {
    const pts = Array.from({ length: 5000 }, (_, i) => ({ x: i, y: Math.sin(i) }));
    expect(decimate(pts, 300).length).toBe(300);
  });

  test('flat signals get a non-zero y range', () => {
    const r = yRangeOf([
      { x: 0, y: 5 },
      { x: 1, y: 5 }
    ]);
    expect(r.min).toBeLessThan(5);
    expect(r.max).toBeGreaterThan(5);
  });

  test('clip space maps the window corners to the square corners and clamps outside values', () => {
    const v = toClipSpace(
      [
        { x: 0, y: 0 },
        { x: 10, y: 100 },
        { x: 20, y: 300 }
      ],
      0,
      10,
      { min: 0, max: 100 }
    );
    expect([...v]).toEqual([-1, -1, 1, 1, 1, 1]);
  });
});
