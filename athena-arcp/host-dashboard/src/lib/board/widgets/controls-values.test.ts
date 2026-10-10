import { describe, expect, test } from 'bun:test';
import type { Sample } from '../../data/types';
import {
  alertTone,
  arcPath,
  fraction,
  gaugeAngle,
  humanLabel,
  matrixColumns,
  numericWindow,
  rowTone,
  sparkPath,
  windowDelta
} from './controls-values';

const S = 1_000_000;
const samples = (pairs: [number, number | string | boolean][]): Sample[] =>
  pairs.map(([t, v]) => ({ timestampMicros: t * S, value: v as Sample['value'] }));

describe('alertTone', () => {
  test('high rules warn then fault', () => {
    const rules = { warn: 60, fault: 80, alertWhen: 'high' as const };
    expect(alertTone(50, rules)).toBeNull();
    expect(alertTone(60, rules)).toBe('warn');
    expect(alertTone(85, rules)).toBe('fault');
  });
  test('low rules alarm when the value falls', () => {
    const rules = { warn: 11.5, fault: 10, alertWhen: 'low' as const };
    expect(alertTone(12, rules)).toBeNull();
    expect(alertTone(11, rules)).toBe('warn');
    expect(alertTone(9.9, rules)).toBe('fault');
  });
  test('blank thresholds and missing values are neutral', () => {
    expect(alertTone(99, { warn: null, fault: Number.NaN, alertWhen: 'high' })).toBeNull();
    expect(alertTone(undefined, { warn: 1, fault: 2, alertWhen: 'high' })).toBeNull();
  });
});

describe('geometry', () => {
  test('fraction clamps and handles degenerate ranges', () => {
    expect(fraction(50, 0, 100)).toBe(0.5);
    expect(fraction(-5, 0, 100)).toBe(0);
    expect(fraction(500, 0, 100)).toBe(1);
    expect(fraction(3, 2, 2)).toBe(0);
    expect(fraction(undefined, 0, 1)).toBe(0);
  });
  test('gauge sweeps -120 to +120 degrees', () => {
    expect(gaugeAngle(0, 0, 100)).toBe(-120);
    expect(gaugeAngle(50, 0, 100)).toBe(0);
    expect(gaugeAngle(100, 0, 100)).toBe(120);
  });
  test('arc path is a valid SVG arc', () => {
    expect(arcPath(50, 50, 40, 0, 0.5)).toMatch(/^M [\d.-]+ [\d.-]+ A 40 40 0 0 1 [\d.-]+ [\d.-]+$/);
    expect(arcPath(50, 50, 40, 0, 0.9)).toContain(' 0 1 1 ');
  });
});

describe('windows and sparklines', () => {
  const now = 100 * S;
  const data = samples([
    [10, 1],
    [95, 4],
    [99, 6],
    [100, 'x' as unknown as number]
  ]);
  test('numeric window drops old and non-numeric samples', () => {
    expect(numericWindow(data, now, 10).map((p) => p.v)).toEqual([4, 6]);
    expect(numericWindow(data, now, 30).map((p) => p.v)).toEqual([4, 6]);
  });
  test('delta is newest minus oldest in window', () => {
    expect(windowDelta(data, now, 10)).toBe(2);
    expect(windowDelta(data, now, 1)).toBeNull();
  });
  test('sparkline needs two points and stays in the box', () => {
    expect(sparkPath(data.slice(0, 1), now, 10, 100, 20)).toBe('');
    const path = sparkPath(data, now, 10, 100, 20);
    expect(path.startsWith('M')).toBe(true);
    expect(path.split(' L')).toHaveLength(2);
    const flat = sparkPath(samples([[95, 3], [99, 3]]), now, 10, 100, 20);
    expect(flat).toContain('10.0');
  });
});

describe('rowTone and layout', () => {
  const rules = {
    warnAbove: 60,
    faultAbove: 80,
    warnBelow: Number.NaN,
    faultBelow: null,
    trueTone: 'ok',
    falseTone: 'off'
  };
  test('numbers use above thresholds, missing data is stale', () => {
    expect(rowTone(40, rules)).toBe('ok');
    expect(rowTone(65, rules)).toBe('warn');
    expect(rowTone(90, rules)).toBe('fault');
    expect(rowTone(undefined, rules)).toBe('stale');
  });
  test('below thresholds and booleans', () => {
    expect(rowTone(2, { ...rules, warnBelow: 5, faultBelow: 1 })).toBe('warn');
    expect(rowTone(0.5, { ...rules, warnBelow: 5, faultBelow: 1 })).toBe('fault');
    expect(rowTone(true, rules)).toBe('ok');
    expect(rowTone(false, rules)).toBe('off');
  });
  test('strings are neutral', () => {
    expect(rowTone('hello', rules)).toBeNull();
  });
  test('matrix columns', () => {
    expect(matrixColumns(9, -1)).toBe(3);
    expect(matrixColumns(10, 0)).toBe(4);
    expect(matrixColumns(10, 2)).toBe(2);
    expect(matrixColumns(0, null)).toBe(1);
  });
  test('human labels', () => {
    expect(humanLabel('shotDistanceMeters')).toBe('Shot distance meters');
    expect(humanLabel('drive_motor_temp')).toBe('Drive motor temp');
    expect(humanLabel('')).toBe('');
  });
});
