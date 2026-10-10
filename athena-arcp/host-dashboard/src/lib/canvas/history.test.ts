import { describe, expect, test } from 'bun:test';
import type { SignalRow } from '../arcp';
import { advanceHistory, collectConfiguredSignalIds, EMPTY_HISTORY, HISTORY_SAMPLES } from './history';

function row(id: number, value: string, type = 'f64'): SignalRow {
  return { signal_id: id, signal_type: type, kind: 'value', access: 'observe', policy: 'periodic', durability: 'persistent', path: `robot/${id}`, value };
}

describe('advanceHistory', () => {
  test('appends numeric values for tracked signals only', () => {
    let state = EMPTY_HISTORY;
    state = advanceHistory(state, [row(1, '1.5'), row(2, '9')], new Set([1]), 100);
    state = advanceHistory(state, [row(1, '2.5'), row(2, '9')], new Set([1]), 200);
    expect(state.values.get(1)).toEqual([1.5, 2.5]);
    expect(state.times.get(1)).toEqual([100, 200]);
    expect(state.values.has(2)).toBe(false);
  });

  test('skips non-numeric signals and forgets signals that left the set', () => {
    let state = advanceHistory(EMPTY_HISTORY, [row(1, 'true', 'bool'), row(2, '3')], new Set([1, 2]), 1);
    expect(state.values.has(1)).toBe(false);
    expect(state.values.get(2)).toEqual([3]);
    state = advanceHistory(state, [row(2, '4')], new Set([1]), 2);
    expect(state.values.has(2)).toBe(false);
  });

  test('caps the window and returns the same object when nothing is tracked', () => {
    let state = EMPTY_HISTORY;
    for (let i = 0; i < HISTORY_SAMPLES + 5; i++) {
      state = advanceHistory(state, [row(1, String(i))], new Set([1]), i);
    }
    const values = state.values.get(1) ?? [];
    expect(values).toHaveLength(HISTORY_SAMPLES);
    expect(values[values.length - 1]).toBe(HISTORY_SAMPLES + 4);
    expect(advanceHistory(EMPTY_HISTORY, [], new Set(), 0)).toBe(EMPTY_HISTORY);
    expect(advanceHistory(state, [], new Set(), 0)).toEqual(EMPTY_HISTORY);
  });
});

describe('collectConfiguredSignalIds', () => {
  test('reads *signalId keys and the known list fields', () => {
    const ids = new Set<number>();
    collectConfiguredSignalIds(
      { primarySignalId: 4, series: [{ signalId: 7 }, { signalId: 0 }], targets: [9.6], other: 99 },
      ids
    );
    expect([...ids].sort((a, b) => a - b)).toEqual([4, 7, 9]);
  });
});
