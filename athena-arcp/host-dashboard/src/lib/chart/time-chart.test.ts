import { describe, expect, test } from 'bun:test';
import type { Sample } from '../data/types';
import { buildChartData, relativeLabel, valueHeldAt, visibleX, yDomainFor } from './time-chart';

const S = (seconds: number, value: number | string): Sample => ({ timestampMicros: Math.round(seconds * 1e6), value });

describe('buildChartData', () => {
  test('aligns series on the union of sample times and leaves gaps as null', () => {
    const a = [S(1, 1), S(2, 2), S(3, 3)];
    const b = [S(2, 20), S(3, 30)];
    const { data, newestSeconds } = buildChartData([{ id: 'a', samples: a }, { id: 'b', samples: b }], 10);
    expect(newestSeconds).toBe(3);
    expect(data[0]).toEqual([1, 2, 3]);
    expect(data[1]).toEqual([1, 2, 3]);
    expect(data[2]).toEqual([null, 20, 30]);
  });

  test('keeps only the trailing window and drops non-numeric samples', () => {
    const samples = [S(0, 5), S(5, 'text'), S(9, 9), S(10, 10)];
    const { data } = buildChartData([{ id: 'a', samples }], 3);
    expect(data[0]).toEqual([9, 10]);
    expect(data[1]).toEqual([9, 10]);
  });

  test('reaches back for older data when a zoom needs it', () => {
    const samples = [S(0, 0), S(5, 5), S(10, 10)];
    const { data } = buildChartData([{ id: 'a', samples }], 2, 0);
    expect(data[0]).toEqual([0, 5, 10]);
  });

  test('empty input still yields a two-point axis', () => {
    const { data } = buildChartData([{ id: 'a', samples: [] }], 10);
    expect(data[0].length).toBe(2);
    expect(data[1]).toEqual([null, null]);
  });
});

describe('yDomainFor', () => {
  const data = [[0, 1, 2], [1, 3, 2]] as const;
  test('auto range covers the visible values with a margin', () => {
    const d = yDomainFor(data as never, { min: 0, max: 2 }, [false], { min: null, max: null });
    expect(d.min).toBeLessThan(1);
    expect(d.max).toBeGreaterThan(3);
  });

  test('ignores points outside the visible x range', () => {
    const d = yDomainFor(data as never, { min: 1, max: 2 }, [false], { min: null, max: null });
    expect(d.max).toBeLessThan(3.5);
  });

  test('hidden series do not count', () => {
    const d = yDomainFor(data as never, { min: 0, max: 2 }, [true], { min: null, max: null });
    expect(d.max).toBeLessThan(1.5);
  });

  test('fixed bounds override the auto side only', () => {
    const d = yDomainFor(data as never, { min: 0, max: 2 }, [false], { min: -5, max: null });
    expect(d.min).toBe(-5);
    expect(d.max).toBeGreaterThan(3);
  });

  test('flat data gets a visible span', () => {
    const flat = [[0, 1], [4, 4]] as const;
    const d = yDomainFor(flat as never, { min: 0, max: 1 }, [false], { min: null, max: null });
    expect(d.max - d.min).toBeGreaterThanOrEqual(0.5);
  });
});

describe('visibleX and relativeLabel', () => {
  test('zoom replaces the trailing window', () => {
    expect(visibleX(100, 10, null)).toEqual({ min: 90, max: 100 });
    expect(visibleX(100, 10, { min: 95, max: 97 })).toEqual({ min: 95, max: 97 });
  });

  test('labels are relative to the newest sample', () => {
    expect(relativeLabel(0)).toBe('now');
    expect(relativeLabel(-10)).toBe('-10 s');
    expect(relativeLabel(-2.345)).toBe('-2.3 s');
  });
});

describe('valueHeldAt', () => {
  test('holds the last value through gaps and is null before the first sample', () => {
    expect(valueHeldAt([null, 2, null, null, 5], 3)).toBe(2);
    expect(valueHeldAt([null, 2], 0)).toBeNull();
    expect(valueHeldAt([1, 2, 3], 2)).toBe(3);
  });
});
