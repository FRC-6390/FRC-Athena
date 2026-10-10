/**
 * Rolling numeric history for classic trend and graph widgets (and the control tuner).
 * Values and timestamps are kept per signal, newest last, capped at HISTORY_SAMPLES.
 */
import type { SignalRow } from '../arcp';
import { parseNumericSignal } from '../dashboard';

export const HISTORY_SAMPLES = 90;

export type HistoryMaps = {
  values: Map<number, number[]>;
  times: Map<number, number[]>;
};

export const EMPTY_HISTORY: HistoryMaps = { values: new Map(), times: new Map() };

/**
 * Appends the current numeric value of every tracked signal and drops signals that are no longer tracked.
 * Returns the previous maps unchanged when nothing needs to change.
 */
export function advanceHistory(
  prev: HistoryMaps,
  signals: readonly SignalRow[],
  tracked: ReadonlySet<number>,
  nowMs: number
): HistoryMaps {
  if (tracked.size === 0) {
    return prev.values.size > 0 || prev.times.size > 0 ? EMPTY_HISTORY : prev;
  }

  const byId = new Map<number, SignalRow>();
  for (const signal of signals) {
    byId.set(signal.signal_id, signal);
  }

  const values = new Map(prev.values);
  const times = new Map(prev.times);
  for (const signalId of [...values.keys()]) {
    if (!tracked.has(signalId)) values.delete(signalId);
  }
  for (const signalId of [...times.keys()]) {
    if (!tracked.has(signalId)) times.delete(signalId);
  }

  for (const signalId of tracked) {
    const signal = byId.get(signalId);
    if (!signal) {
      values.delete(signalId);
      times.delete(signalId);
      continue;
    }
    const numeric = parseNumericSignal(signal);
    if (numeric === null) continue;

    const nextValues = [...(values.get(signalId) ?? []), numeric];
    const nextTimes = [...(times.get(signalId) ?? []), nowMs];
    values.set(signalId, nextValues.slice(-HISTORY_SAMPLES));
    times.set(signalId, nextTimes.slice(-HISTORY_SAMPLES));
  }

  return { values, times };
}

/** Collects the signal ids a widget config references (keys ending in `signalId` and known list fields). */
export function collectConfiguredSignalIds(value: unknown, out: Set<number>): void {
  if (value === null || value === undefined) return;
  if (typeof value === 'number') {
    if (Number.isFinite(value) && value > 0) {
      out.add(Math.floor(value));
    }
    return;
  }
  if (Array.isArray(value)) {
    for (const item of value) {
      collectConfiguredSignalIds(item, out);
    }
    return;
  }
  if (typeof value !== 'object') {
    return;
  }
  const record = value as Record<string, unknown>;
  for (const [key, nested] of Object.entries(record)) {
    if (key.toLowerCase().endsWith('signalid')) {
      collectConfiguredSignalIds(nested, out);
      continue;
    }
    if (key === 'series' || key === 'actions' || key === 'columns' || key === 'signals' || key === 'targets') {
      collectConfiguredSignalIds(nested, out);
    }
  }
}
