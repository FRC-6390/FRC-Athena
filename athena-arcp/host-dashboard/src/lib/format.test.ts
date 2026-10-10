import { describe, expect, test } from 'bun:test';
import { formatNumber, formatValue, parseInput } from './format';

describe('parseInput', () => {
  test('accepts numbers as text or as number-input values', () => {
    expect(parseInput('f64', '0.5')).toBe(0.5);
    expect(parseInput('f64', 0.5)).toBe(0.5);
    expect(parseInput('i64', '7.9')).toBe(7);
  });

  test('rejects empty and non-finite numbers', () => {
    expect(parseInput('f64', '')).toBeNull();
    expect(parseInput('f64', 'abc')).toBeNull();
    expect(parseInput('f64', Number.NaN)).toBeNull();
  });

  test('parses booleans and passes strings through', () => {
    expect(parseInput('bool', 'true')).toBe(true);
    expect(parseInput('bool', '0')).toBe(false);
    expect(parseInput('bool', 'maybe')).toBeNull();
    expect(parseInput('string', ' hi ')).toBe(' hi ');
  });
});

describe('formatValue', () => {
  test('formats numbers by magnitude and arrays inline', () => {
    expect(formatNumber(3)).toBe('3');
    expect(formatNumber(1.23456)).toBe('1.235');
    expect(formatNumber(12.34)).toBe('12.3');
    expect(formatValue([1, 2.5])).toBe('[1, 2.500]');
    expect(formatValue(undefined)).toBe('--');
    expect(formatValue('')).toBe('""');
  });
});

import { formatSignalValue } from './format';
import { encodePose2d } from './data/mock-robot';

test('formatSignalValue decodes Pose2d structs', () => {
  const text = formatSignalValue('struct:Pose2d', { schemaId: 1, bytes: encodePose2d(1.5, 2.25, Math.PI / 2) });
  expect(text).toBe('1.50, 2.25 m · 90.0°');
});

test('formatSignalValue falls back for unknown structs', () => {
  expect(formatSignalValue('struct:Other', { schemaId: 3, bytes: new Uint8Array(8) })).toBe('struct #3 (8 B)');
});
