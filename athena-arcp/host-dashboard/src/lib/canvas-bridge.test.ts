import { describe, expect, test } from 'bun:test';
import type { CommandReply } from './data/commands';
import type { Sample, SignalDescriptor } from './data/types';
import {
  classicAccess,
  classicCommands,
  classicValueText,
  parseClassicValue,
  toClassicRow,
  toClassicRows,
  type CanvasStoreView
} from './canvas-bridge';

function descriptor(overrides: Partial<SignalDescriptor> & Pick<SignalDescriptor, 'signalId' | 'path' | 'type'>): SignalDescriptor {
  return {
    typeTag: 3,
    unit: '',
    role: 'value',
    writable: false,
    persisted: true,
    lockGroup: '',
    structSchemaId: 0,
    mechanismPath: 'robot/turret',
    localPath: '',
    name: overrides.path.split('/').pop() ?? '',
    ...overrides
  };
}

function fakeStore(signals: SignalDescriptor[], samples: Record<number, Sample> = {}): CanvasStoreView {
  const byId = new Map(signals.map((s) => [s.signalId, s]));
  return {
    allSignals: () => [...signals].sort((a, b) => a.path.localeCompare(b.path)),
    latestSample: (id) => samples[id],
    descriptor: (id) => byId.get(id)
  } as CanvasStoreView;
}

const reply = (op: CommandReply['op']): CommandReply => ({
  requestId: 1,
  op,
  signalId: 0,
  accepted: true,
  reason: '',
  actualValue: null,
  receivedAtMs: 0
});

describe('classicAccess', () => {
  test('maps role and writability to the classic access strings', () => {
    expect(classicAccess(descriptor({ signalId: 1, path: 'a/x', type: 'f64', role: 'action' }))).toBe('invoke');
    expect(classicAccess(descriptor({ signalId: 2, path: 'a/y', type: 'f64', writable: true }))).toBe('write');
    expect(classicAccess(descriptor({ signalId: 3, path: 'a/z', type: 'f64' }))).toBe('observe');
  });
});

describe('classicValueText', () => {
  test('renders scalars, arrays and missing values for the classic widgets', () => {
    expect(classicValueText(undefined)).toBe('-');
    expect(classicValueText(true)).toBe('true');
    expect(classicValueText(0.125)).toBe('0.125');
    expect(classicValueText(7n)).toBe('7');
    expect(classicValueText([1, 2.5])).toBe('[1,2.5]');
    expect(classicValueText(['a', 'b'])).toBe('["a","b"]');
    expect(classicValueText({ schemaId: 4, bytes: new Uint8Array(3) })).toBe('struct #4 (3 B)');
  });
});

describe('toClassicRow', () => {
  test('fills classic defaults from the descriptor', () => {
    const row = toClassicRow(
      descriptor({ signalId: 9, path: 'robot/turret/Controls/kP', type: 'f64', writable: true, persisted: false }),
      { timestampMicros: 1, value: 1.5 }
    );
    expect(row).toEqual({
      signal_id: 9,
      signal_type: 'f64',
      kind: 'value',
      access: 'write',
      policy: 'periodic',
      durability: 'volatile',
      path: 'robot/turret/Controls/kP',
      value: '1.5'
    });
  });

  test('bool signals are on_change and actions are commands', () => {
    const flag = toClassicRow(descriptor({ signalId: 1, path: 'robot/a/flag', type: 'bool' }), undefined);
    expect(flag.policy).toBe('on_change');
    expect(flag.value).toBe('-');
    const action = toClassicRow(descriptor({ signalId: 2, path: 'robot/a/go', type: 'bool', role: 'action' }), undefined);
    expect(action.kind).toBe('command');
    expect(action.access).toBe('invoke');
  });

  test('toClassicRows follows store order and reads the latest sample', () => {
    const store = fakeStore(
      [descriptor({ signalId: 2, path: 'robot/b', type: 'i64' }), descriptor({ signalId: 1, path: 'robot/a', type: 'i64' })],
      { 1: { timestampMicros: 1, value: 3 } }
    );
    const rows = toClassicRows(store);
    expect(rows.map((r) => [r.path, r.value])).toEqual([
      ['robot/a', '3'],
      ['robot/b', '-']
    ]);
  });
});

describe('parseClassicValue', () => {
  test('parses scalars with the shared input parser', () => {
    expect(parseClassicValue('f64', '0.5')).toBe(0.5);
    expect(parseClassicValue('i64', '7.9')).toBe(7);
    expect(parseClassicValue('bool', 'true')).toBe(true);
    expect(parseClassicValue('f64', '')).toBeNull();
  });

  test('parses JSON arrays element by element', () => {
    expect(parseClassicValue('f64[]', '[1, 2.5]')).toEqual([1, 2.5]);
    expect(parseClassicValue('string[]', '["a","b"]')).toEqual(['a', 'b']);
    expect(parseClassicValue('f64[]', '3')).toBeNull();
    expect(parseClassicValue('f64[]', '[1,"x"]')).toBeNull();
    expect(parseClassicValue('f64[]', 'nope')).toBeNull();
  });
});

describe('classicCommands', () => {
  test('setValue parses by descriptor type and writes through the session', async () => {
    const calls: unknown[][] = [];
    const store = fakeStore([descriptor({ signalId: 5, path: 'robot/x', type: 'i64', writable: true })]);
    const commands = classicCommands(store, {
      write: async (id, value) => {
        calls.push(['write', id, value]);
        return reply('write');
      },
      run: async (id) => {
        calls.push(['run', id]);
        return reply('run');
      }
    });
    expect(await commands.setValue(5, '12.7')).toBeNull();
    expect(calls).toEqual([['write', 5, 12]]);
  });

  test('invalid text and unknown signals send nothing', async () => {
    const calls: unknown[][] = [];
    const store = fakeStore([descriptor({ signalId: 5, path: 'robot/x', type: 'f64', writable: true })]);
    const commands = classicCommands(store, {
      write: async () => {
        calls.push(['write']);
        return reply('write');
      },
      run: async () => reply('run')
    });
    expect(await commands.setValue(5, 'abc')).toContain('not a valid f64');
    expect(await commands.setValue(99, '1')).toContain('unknown signal');
    expect(calls).toEqual([]);
  });

  test('triggerAction runs the signal and reports robot rejections', async () => {
    const calls: number[] = [];
    const commands = classicCommands(fakeStore([]), {
      write: async () => reply('write'),
      run: async (id) => {
        calls.push(id);
        return id === 42 ? reply('run') : { ...reply('run'), accepted: false, reason: 'rejected: locked (InMatch)' };
      }
    });
    expect(await commands.triggerAction(42)).toBeNull();
    expect(await commands.triggerAction(43)).toBe('rejected: locked (InMatch)');
    expect(calls).toEqual([42, 43]);
  });
});
