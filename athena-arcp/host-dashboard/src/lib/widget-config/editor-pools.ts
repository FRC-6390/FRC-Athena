import type { SignalRow } from '../arcp';
import { leafPath, normalizeToken } from './editor-helpers';

/** Signals grouped by what the config editor offers them for. Each list is sorted by path. */
export type EditorPools = {
  numericSignals: SignalRow[];
  boolSignals: SignalRow[];
  stringSignals: SignalRow[];
  writableNumericSignals: SignalRow[];
  writableBoolSignals: SignalRow[];
  vectorSignals: SignalRow[];
  actionSignals: SignalRow[];
  /** Bool and string signals usable as a mode selector. */
  modeSignals: SignalRow[];
  writableMotorCommandSignals: SignalRow[];
  preferredMotorCommandSignals: SignalRow[];
};

const byPath = (a: SignalRow, b: SignalRow) => a.path.localeCompare(b.path);

/** Motor command-like names. Current limits and CAN ids are excluded. */
export function isMotorCommandName(path: string): boolean {
  const token = normalizeToken(leafPath(path));
  if (!token) return false;
  if (token.includes('currentlimit') || token.includes('limitamps')) return false;
  if (token.includes('canid')) return false;
  return [
    'command',
    'setpoint',
    'target',
    'output',
    'percent',
    'duty',
    'voltage',
    'volt',
    'velocity',
    'speed',
    'rpm'
  ].some((word) => token.includes(word));
}

export function buildEditorPools(signals: readonly SignalRow[]): EditorPools {
  const numericSignals = signals
    .filter((signal) => signal.signal_type === 'f64' || signal.signal_type === 'i64')
    .sort(byPath);
  const boolSignals = signals.filter((signal) => signal.signal_type === 'bool').sort(byPath);
  const stringSignals = signals
    .filter((signal) => signal.signal_type === 'string' || signal.signal_type === 'string[]')
    .sort(byPath);
  const writableNumericSignals = numericSignals.filter((signal) => signal.access === 'write');
  const writableMotorCommandSignals = writableNumericSignals.filter((signal) => isMotorCommandName(signal.path));
  return {
    numericSignals,
    boolSignals,
    stringSignals,
    writableNumericSignals,
    writableBoolSignals: boolSignals.filter((signal) => signal.access === 'write'),
    vectorSignals: signals
      .filter((signal) => signal.signal_type === 'f64[]' || signal.signal_type === 'i64[]')
      .sort(byPath),
    actionSignals: signals
      .filter((signal) => signal.access === 'invoke' || signal.kind === 'command')
      .sort(byPath),
    modeSignals: signals
      .filter(
        (signal) =>
          signal.signal_type === 'bool' || signal.signal_type === 'string' || signal.signal_type === 'string[]'
      )
      .sort(byPath),
    writableMotorCommandSignals,
    preferredMotorCommandSignals:
      writableMotorCommandSignals.length > 0 ? writableMotorCommandSignals : writableNumericSignals
  };
}
