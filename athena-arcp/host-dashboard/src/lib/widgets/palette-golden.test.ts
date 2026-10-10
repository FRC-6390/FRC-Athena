import { describe, expect, test } from 'bun:test';
import paletteGolden from '../canvas/fixtures/palette-golden.json';
import { widgetKindsFor, type SignalRow } from '../dashboard';
import type { SignalRow as ArcpSignalRow } from '../arcp';

// Golden palette data recorded from the pre-split `widgetKindsFor`. The registry must keep offering the
// same classic widgets for every signal shape (order inside a signal's list is not significant).

const TYPES = ['bool', 'i64', 'f64', 'string', 'bool[]', 'i64[]', 'f64[]', 'string[]'];
const PATHS = ['robot/turret/Values/x', 'robot/shooter/motor/output', 'robot/shooter/Hooks/ready', 'robot/climber/encoder/pos'];

function matrixRows(): ArcpSignalRow[] {
  const rows: ArcpSignalRow[] = [];
  for (const signal_type of TYPES)
    for (const access of ['observe', 'write', 'invoke'])
      for (const kind of ['value', 'command'])
        for (const policy of ['on_change', 'periodic'])
          for (const durability of ['volatile', 'persistent'])
            for (const path of PATHS) {
              rows.push({ signal_id: 100 + rows.length, path, signal_type, access, kind, policy, durability, value: '1' });
            }
  return rows;
}

const sorted = (values: readonly string[]) => [...values].sort();

describe('classic palette golden (pre-split widgetKindsFor)', () => {
  test('no-signal palette is unchanged', () => {
    expect(widgetKindsFor(null)).toEqual(paletteGolden.noSignal as never);
  });

  test('every recorded signal shape offers the same widgets', () => {
    for (const entry of paletteGolden.perSignal) {
      expect(sorted(widgetKindsFor(entry.row as SignalRow))).toEqual(sorted(entry.kinds));
    }
  });

  test('the full signal matrix offers the same widgets', () => {
    const rows = matrixRows();
    expect(rows.length).toBe(paletteGolden.matrix.length);
    rows.forEach((row, index) => {
      expect(sorted(widgetKindsFor(row as SignalRow))).toEqual(paletteGolden.matrix[index]);
    });
  });
});
