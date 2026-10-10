import { describe, expect, test } from 'bun:test';
import type { Sample } from '../../data/types';
import { alignPairs, decimateStride, minMaxColumns, paddedDomain, rollingStats, windowedNumeric, xyDomain } from './robot-series';

const S = (seconds: number, value: number | string | boolean): Sample => ({ timestampMicros: Math.round(seconds * 1e6), value: value as Sample['value'] });

describe('windowing', () => {
  test('keeps numeric samples in the window, relative to now', () => {
    const samples = [S(0, 1), S(5, 2), S(9, 'x'), S(10, 3)];
    const out = windowedNumeric(samples, 10e6, 6);
    expect(out).toEqual([
      { t: -5, v: 2 },
      { t: 0, v: 3 }
    ]);
  });
  test('bigint values are converted', () => {
    expect(windowedNumeric([{ timestampMicros: 1e6, value: 12n }], 1e6, 5)).toEqual([{ t: 0, v: 12 }]);
  });
});

describe('pairing', () => {
  test('zero-order hold matches the latest y at or before each x', () => {
    const xs = [S(1, 10), S(2, 20), S(3, 30)];
    const ys = [S(0.5, 100), S(2.5, 200)];
    expect(alignPairs(xs, ys, 3e6, 10)).toEqual([
      { x: 10, y: 100, t: -2 },
      { x: 20, y: 100, t: -1 },
      { x: 30, y: 200, t: 0 }
    ]);
  });
  test('x samples before any y sample are skipped', () => {
    expect(alignPairs([S(1, 1)], [S(2, 5)], 3e6, 10)).toEqual([]);
  });
});

describe('decimation', () => {
  test('stride keeps the newest item and never exceeds the cap', () => {
    const items = Array.from({ length: 1000 }, (_, i) => i);
    const out = decimateStride(items, 50);
    expect(out.length).toBe(50);
    expect(out[out.length - 1]).toBe(999);
    expect(decimateStride([1, 2], 10)).toEqual([1, 2]);
  });
  test('min/max columns bound the output and keep extremes', () => {
    const values = Array.from({ length: 1000 }, (_, i) => ({ t: i / 1000, v: i === 500 ? 99 : 0 }));
    const cols = minMaxColumns(values, 10, 0, 1);
    expect(cols.length).toBe(10);
    expect(Math.max(...cols.map((c) => c.max))).toBe(99);
    expect(minMaxColumns(values, 10, 1, 1)).toEqual([]);
  });
});

describe('domains', () => {
  test('flat signals get a minimum span around their value', () => {
    expect(paddedDomain(5, 5, { minSpan: 1 })).toEqual({ min: 4.5, max: 5.5 });
  });
  test('padding and fixed bounds', () => {
    const d = paddedDomain(0, 10, { minSpan: 1, fixedMin: -1 });
    expect(d.min).toBe(-1);
    expect(d.max).toBeCloseTo(10.8);
  });
  test('equal axes widen the shorter axis', () => {
    const d = xyDomain([{ x: 0, y: 0, t: 0 }, { x: 10, y: 1, t: 0 }], true);
    expect(d.y.max - d.y.min).toBeCloseTo(d.x.max - d.x.min);
  });
});

describe('rolling stats', () => {
  test('min, max, mean, std, rate and histogram', () => {
    const samples = [S(0, 1), S(1, 2), S(2, 3), S(3, 4), S(4, 5)];
    const s = rollingStats(samples, 4e6, 10, 4);
    expect(s).not.toBeNull();
    expect(s!.count).toBe(5);
    expect(s!.min).toBe(1);
    expect(s!.max).toBe(5);
    expect(s!.mean).toBeCloseTo(3);
    expect(s!.std).toBeCloseTo(Math.sqrt(2));
    expect(s!.rateHz).toBeCloseTo(1);
    expect(s!.histogram.reduce((a, b) => a + b, 0)).toBe(5);
  });
  test('no numeric data gives null, flat data gives an empty histogram', () => {
    expect(rollingStats([S(0, 'a')], 1e6, 5)).toBeNull();
    const flat = rollingStats([S(0, 2), S(1, 2)], 1e6, 5, 8);
    expect(flat!.histogram).toEqual([]);
    expect(flat!.rateHz).toBeCloseTo(1);
  });
});
