import { describe, expect, test } from 'bun:test';
import { clamp, confirmReason, decimalsOf, describeChange, finiteSetting, nudge, parseNumberInput, safeRange } from './controls-numeric';

describe('safeRange', () => {
  test('defaults to a window around the current value', () => {
    expect(safeRange(10)).toEqual({ min: 0, max: 40 });
    expect(safeRange(0)).toEqual({ min: 0, max: 1 });
    expect(safeRange(-5)).toEqual({ min: -20, max: 20 });
  });
  test('unknown current value gives the small default window', () => {
    expect(safeRange(undefined)).toEqual({ min: 0, max: 1 });
    expect(safeRange(Number.NaN)).toEqual({ min: 0, max: 1 });
  });
  test('user bounds win, and null (blank setting) is ignored', () => {
    expect(safeRange(10, 2, 8)).toEqual({ min: 2, max: 8 });
    expect(safeRange(10, null, 50)).toEqual({ min: 0, max: 50 });
  });
  test('reversed user bounds are swapped', () => {
    expect(safeRange(1, 9, 3)).toEqual({ min: 3, max: 9 });
  });
});

describe('steps and parsing', () => {
  test('decimals follow the step', () => {
    expect(decimalsOf(1)).toBe(0);
    expect(decimalsOf(0.25)).toBe(2);
    expect(decimalsOf(1e-7)).toBe(6);
    expect(decimalsOf(0)).toBe(0);
  });
  test('nudge avoids float drift and clamps', () => {
    expect(nudge(0.1, 1, 0.2)).toBe(0.3);
    expect(nudge(0.3, -1, 0.1)).toBe(0.2);
    expect(nudge(9.5, 1, 1, { min: 0, max: 10 })).toBe(10);
    expect(nudge(1, -1, 0)).toBe(0);
  });
  test('parseNumberInput rejects blank and non-finite text', () => {
    expect(parseNumberInput(' 12.5 ')).toBe(12.5);
    expect(parseNumberInput('')).toBeNull();
    expect(parseNumberInput('abc')).toBeNull();
    expect(parseNumberInput('Infinity')).toBeNull();
    expect(parseNumberInput(null)).toBeNull();
    expect(parseNumberInput(7)).toBe(7);
  });
  test('clamp and finiteSetting', () => {
    expect(clamp(5, undefined)).toBe(5);
    expect(clamp(-1, { min: 0, max: 3 })).toBe(0);
    expect(finiteSetting(null)).toBeNull();
    expect(finiteSetting(Number.NaN)).toBeNull();
    expect(finiteSetting(2)).toBe(2);
  });
});

describe('confirmReason', () => {
  test('small changes go straight out', () => {
    expect(confirmReason(10, 14)).toBeNull();
    expect(confirmReason(10, 15)).toBeNull();
    expect(confirmReason(10, 10)).toBeNull();
  });
  test('changes over 50% need confirmation, both directions', () => {
    expect(confirmReason(10, 16)).toBe('change');
    expect(confirmReason(10, 4)).toBe('change');
    expect(confirmReason(-10, -2)).toBe('change');
  });
  test('any non-zero value from zero needs confirmation', () => {
    expect(confirmReason(0, 0.001)).toBe('change');
    expect(confirmReason(0, 0)).toBeNull();
  });
  test('unknown robot value needs confirmation', () => {
    expect(confirmReason(undefined, 3)).toBe('unknown');
    expect(confirmReason(null, 3)).toBe('unknown');
  });
  test('user bounds are checked first', () => {
    expect(confirmReason(10, 12, { userMin: 0, userMax: 11 })).toBe('range');
    expect(confirmReason(10, 12, { userMin: 0, userMax: 20 })).toBeNull();
    expect(confirmReason(10, 12, { userMin: null, userMax: null })).toBeNull();
  });
  test('describeChange reads well', () => {
    expect(describeChange(10, 30)).toBe('10 to 30 (+200%)');
    expect(describeChange(20, 5)).toBe('20 to 5 (-75%)');
    expect(describeChange(0, 4)).toBe('from 0 to 4');
    expect(describeChange(undefined, 4)).toBe('to 4');
  });
});
