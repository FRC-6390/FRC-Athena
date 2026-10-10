import { describe, expect, test } from 'bun:test';
import { lttb, type Point } from './lttb';

const line = (n: number): Point[] => Array.from({ length: n }, (_, i) => ({ x: i, y: i * 2 }));

describe('lttb', () => {
  test('returns the input unchanged when it is already within the threshold', () => {
    const pts = line(10);
    expect(lttb(pts, 10)).toEqual(pts);
    expect(lttb(pts, 50)).toEqual(pts);
  });

  test('keeps the endpoints and yields exactly the threshold count', () => {
    const pts = Array.from({ length: 1000 }, (_, i) => ({ x: i, y: Math.sin(i / 10) }));
    const out = lttb(pts, 100);
    expect(out).toHaveLength(100);
    expect(out[0]).toEqual(pts[0]);
    expect(out[out.length - 1]).toEqual(pts[pts.length - 1]);
  });

  test('output x values stay sorted and are a subset of the input', () => {
    const pts = Array.from({ length: 500 }, (_, i) => ({ x: i * 0.5, y: (i * 37) % 11 }));
    const out = lttb(pts, 40);
    for (let i = 1; i < out.length; i++) expect(out[i].x).toBeGreaterThan(out[i - 1].x);
    const xs = new Set(pts.map((p) => p.x));
    expect(out.every((p) => xs.has(p.x))).toBe(true);
  });

  test('a single spike survives decimation', () => {
    const pts = Array.from({ length: 2000 }, (_, i) => ({ x: i, y: i === 1234 ? 100 : 0 }));
    const out = lttb(pts, 50);
    expect(out.some((p) => p.y === 100)).toBe(true);
  });

  test('does not mutate the input and rejects tiny thresholds by copying', () => {
    const pts = line(20);
    const copy = pts.slice();
    const out = lttb(pts, 2);
    expect(out).toEqual(copy);
    expect(out).not.toBe(pts);
    expect(pts).toEqual(copy);
  });
});
