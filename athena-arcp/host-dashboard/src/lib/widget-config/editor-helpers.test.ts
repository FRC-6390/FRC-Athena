import { describe, expect, test } from 'bun:test';
import type { SignalRow } from '../arcp';
import { buildEditorPools, isMotorCommandName } from './editor-pools';
import {
  appendControllerParam,
  nextControllerKey,
  normalizeControllerKey,
  parseOptionalNumber,
  patchControllerParam,
  removeControllerParamAt,
  withSeriesColor,
  withSeriesStyle,
  withoutSeries
} from './editor-helpers';
import type { ControllerWidgetConfig } from './controller';
import type { GraphWidgetConfig } from './graph';

function row(id: number, path: string, signal_type: string, access = 'observe', kind = 'value'): SignalRow {
  return { signal_id: id, path, signal_type, access, kind, policy: 'periodic', durability: 'persistent', value: '0' };
}

const controller = (params: ControllerWidgetConfig['params']): ControllerWidgetConfig =>
  ({ params, autotune: {} }) as unknown as ControllerWidgetConfig;

describe('controller parameter edits', () => {
  test('keys are normalised and de-duplicated with a numeric suffix', () => {
    expect(normalizeControllerKey('  Feed Forward!! ')).toBe('feed_forward');
    expect(normalizeControllerKey('***')).toBe('value');
    const existing = [{ key: 'kp', label: 'kP', signalId: 1 }, { key: 'kp_2', label: 'kP', signalId: 2 }];
    expect(nextControllerKey(existing, 'kP')).toBe('kp_3');
    expect(nextControllerKey(existing, 'kI')).toBe('ki');
  });

  test('patching changes only the addressed parameter', () => {
    const c = controller([
      { key: 'kp', label: 'kP', signalId: 1 },
      { key: 'ki', label: 'kI', signalId: 2 }
    ]);
    const next = patchControllerParam(c, 1, { label: 'Integral' });
    expect(next.params).toEqual([
      { key: 'kp', label: 'kP', signalId: 1 },
      { key: 'ki', label: 'Integral', signalId: 2 }
    ]);
  });

  test('the last parameter cannot be removed', () => {
    const c = controller([{ key: 'kp', label: 'kP', signalId: 1 }]);
    expect(removeControllerParamAt(c, 0)).toBeNull();
    const two = controller([
      { key: 'kp', label: 'kP', signalId: 1 },
      { key: 'kd', label: 'kD', signalId: 3 }
    ]);
    expect(removeControllerParamAt(two, 0)?.params.map((p) => p.key)).toEqual(['kd']);
  });

  test('appending uses the leaf name and refuses a signal already in use', () => {
    const c = controller([{ key: 'kp', label: 'kP', signalId: 1 }]);
    const added = appendControllerParam(c, { signal_id: 7, path: 'robot/arm/Controls/kP' });
    expect(added?.params[1]).toEqual({ key: 'kp_2', label: 'kP', signalId: 7 });
    expect(appendControllerParam(c, { signal_id: 1, path: 'robot/arm/Controls/kP' })).toBeNull();
  });
});

describe('graph series edits', () => {
  const graph = {
    showLegend: true,
    yMin: null,
    yMax: null,
    series: [
      { signalId: 1, style: 'line', color: '#111111' },
      { signalId: 2, style: 'step', color: '#222222' }
    ]
  } as unknown as GraphWidgetConfig;

  test('style and colour change one series', () => {
    expect(withSeriesStyle(graph, 2, 'dot').series[1].style).toBe('dot');
    expect(withSeriesStyle(graph, 2, 'dot').series[0].style).toBe('line');
    expect(withSeriesColor(graph, 1, '#abcdef').series[0].color).toBe('#abcdef');
  });

  test('the last series cannot be removed', () => {
    expect(withoutSeries(graph, 1)?.series.map((s) => s.signalId)).toEqual([2]);
    expect(withoutSeries({ ...graph, series: [graph.series[0]] }, 1)).toBeNull();
  });

  test('axis limits: blank or non-finite input means no limit', () => {
    expect(parseOptionalNumber('  ')).toBeNull();
    expect(parseOptionalNumber('abc')).toBeNull();
    expect(parseOptionalNumber('-2.5')).toBe(-2.5);
  });
});

describe('editor signal pools', () => {
  const signals = [
    row(1, 'robot/arm/Controls/kP', 'f64', 'write'),
    row(2, 'robot/arm/Controls/currentLimitAmps', 'f64', 'write'),
    row(3, 'robot/arm/motor/Output', 'f64', 'write'),
    row(4, 'robot/arm/Hooks/ready', 'bool'),
    row(5, 'robot/arm/Actions/home', 'bool', 'invoke', 'command'),
    row(6, 'robot/arm/Values/pose', 'f64[]')
  ];
  const pools = buildEditorPools(signals);

  test('pools split signals by type and access, sorted by path', () => {
    expect(pools.numericSignals.map((s) => s.signal_id)).toEqual([2, 1, 3]);
    expect(pools.writableNumericSignals.map((s) => s.signal_id)).toEqual([2, 1, 3]);
    expect(pools.boolSignals.map((s) => s.signal_id)).toEqual([4, 5]);
    expect(pools.vectorSignals.map((s) => s.signal_id)).toEqual([6]);
    expect(pools.actionSignals.map((s) => s.signal_id)).toEqual([5]);
    expect(pools.writableBoolSignals).toEqual([]);
  });

  test('motor command candidates skip current limits and fall back to all writable numbers', () => {
    expect(isMotorCommandName('robot/arm/motor/Output')).toBe(true);
    expect(isMotorCommandName('robot/arm/Controls/currentLimitAmps')).toBe(false);
    expect(pools.writableMotorCommandSignals.map((s) => s.signal_id)).toEqual([3]);
    expect(pools.preferredMotorCommandSignals.map((s) => s.signal_id)).toEqual([3]);
    const noCommand = buildEditorPools([row(2, 'robot/arm/Controls/currentLimitAmps', 'f64', 'write')]);
    expect(noCommand.preferredMotorCommandSignals.map((s) => s.signal_id)).toEqual([2]);
  });
});
