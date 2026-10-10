import { describe, expect, test } from 'bun:test';
import type { SignalRow } from '../arcp';
import { createSignalResolver, normalizeSignalPathForLookup } from './signal-resolve';
import { at, widget } from './test-support';

function row(id: number, path: string, value = '1'): SignalRow {
  return { signal_id: id, path, signal_type: 'f64', kind: 'value', access: 'observe', policy: 'periodic', durability: 'persistent', value };
}

const signals = [
  row(1, 'robot/turret/Controls/position/measured', '12.5'),
  row(2, 'robot/turret/Controls/position/target', '-'),
  row(3, 'robot/turret/Values/target', '4'),
  row(4, 'robot/drive/State/heading', '90')
];
const signalById = new Map(signals.map((s) => [s.signal_id, s] as const));
const resolver = createSignalResolver(signals, signalById);

describe('signal resolution for classic widgets', () => {
  test('a widget bound by id resolves to that signal', () => {
    expect(resolver.resolveWidgetSignal(widget('w', 'metric', at(1, 1, 2, 1), { signalId: 4 }))?.path).toBe(
      'robot/drive/State/heading'
    );
  });

  test('layout blocks never resolve to a signal', () => {
    expect(resolver.resolveWidgetSignal(widget('l', 'layout_list', at(1, 1, 2, 1), { signalId: 4 }))).toBeNull();
  });

  test('a signal path in config is resolved relative to the topic, tolerant of spelling', () => {
    const w = widget('w', 'metric', at(1, 1, 2, 1), { config: { topicPath: 'Robot/Turret', signalPath: 'Values/Target' } });
    expect(resolver.resolveWidgetSignal(w)?.signal_id).toBe(3);
  });

  test('inside a topic, a bound signal whose leaf matches the preferred leaf is kept', () => {
    // signal 2 (`target`, placeholder) is bound; the topic has no live `target` elsewhere, so it stays.
    const same = widget('w', 'metric', at(1, 1, 2, 1), {
      signalId: 2,
      config: { topicPath: 'robot/turret/controls/position' }
    });
    expect(resolver.resolveWidgetSignal(same)?.signal_id).toBe(2);
    // signal 1 (`measured`, live) is bound: a live direct match is kept as is.
    const live = widget('w', 'metric', at(1, 1, 2, 1), {
      signalId: 1,
      config: { topicPath: 'robot/turret/controls/position' }
    });
    expect(resolver.resolveWidgetSignal(live)?.signal_id).toBe(1);
  });

  test('signal ids nested in config (graph series) prefer the first live signal over placeholders', () => {
    const w = widget('g', 'graph', at(1, 1, 4, 2), {
      config: { series: [{ signalId: 2 }, { signalId: 4 }] }
    });
    expect(resolver.resolveWidgetSignal(w)?.signal_id).toBe(4);
  });

  test('an unresolvable widget reports its best hint', () => {
    expect(resolver.resolveWidgetSignal(widget('x', 'metric', at(1, 1, 2, 1), { signalId: 99 }))).toBeNull();
    expect(resolver.unavailableSignalHint(widget('x', 'metric', at(1, 1, 2, 1), { signalId: 99 }))).toBe('signal #99');
    expect(
      resolver.unavailableSignalHint(widget('x', 'metric', at(1, 1, 2, 1), { config: { signalPath: 'robot/none' } }))
    ).toBe('robot/none');
    expect(resolver.unavailableSignalHint(widget('x', 'metric', at(1, 1, 2, 1)))).toBe('no bound signal');
  });

  test('path normalisation folds slashes and case', () => {
    expect(normalizeSignalPathForLookup('\\Robot//Turret/ ')).toBe('robot/turret');
  });
});
