import { describe, expect, test, beforeEach } from 'bun:test';
import type { SignalDescriptor } from '../data/types';
import { clearRegistry, createWidget, matches, registerWidget, suggestWidgets } from './registry';
import { exportBoards, importBoards, loadBoards, saveBoards, type KeyValueStorage } from './storage';
import { History } from './history';
import type { BoardDoc, WidgetDef } from './types';
import { driverBoard } from './templates';

function signal(partial: Partial<SignalDescriptor>): SignalDescriptor {
  return {
    signalId: 1,
    path: 'robot/turret/Values/x',
    type: 'f64',
    typeTag: 3,
    unit: '',
    role: 'value',
    writable: false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0,
    mechanismPath: 'robot/turret',
    localPath: 'Values/x',
    name: 'x',
    ...partial
  } as SignalDescriptor;
}

function def(partial: Partial<WidgetDef> & Pick<WidgetDef, 'type'>): WidgetDef {
  return {
    name: partial.type,
    category: 'Values',
    description: '',
    slots: [{ key: 'value', label: 'Signal', matcher: {}, required: true }],
    settings: [],
    defaultSize: { w: 4, h: 2 },
    component: (() => null) as unknown as WidgetDef['component'],
    ...partial
  };
}

describe('registry', () => {
  beforeEach(() => clearRegistry());

  test('matchers check type families, writability and actions', () => {
    expect(matches({ types: ['f64'] }, signal({}))).toBe(true);
    expect(matches({ types: ['struct:*'] }, signal({ type: 'struct:Pose2d' }))).toBe(true);
    expect(matches({ types: ['bool'] }, signal({}))).toBe(false);
    expect(matches({ writable: true }, signal({}))).toBe(false);
    expect(matches({ action: true }, signal({ localPath: 'Actions/go' }))).toBe(true);
  });

  test('suggestions are filtered by slot and ordered by score', () => {
    const value = def({ type: 'value', score: () => 10 });
    const graph = def({ type: 'graph', score: () => 40, slots: [{ key: 's', label: 's', matcher: { types: ['f64'] }, required: true }] });
    const light = def({ type: 'light', score: () => 50, slots: [{ key: 'v', label: 'v', matcher: { types: ['bool'] }, required: true }] });
    [value, graph, light].forEach(registerWidget);
    expect(suggestWidgets(signal({})).map((d) => d.type)).toEqual(['graph', 'value']);
    expect(suggestWidgets(signal({ type: 'bool' })).map((d) => d.type)).toEqual(['light', 'value']);
  });

  test('a new widget binds the dropped signal to its primary slot', () => {
    const graph = def({ type: 'graph', slots: [{ key: 'series', label: 's', matcher: { types: ['f64'] }, multiple: true, required: true }] });
    const instance = createWidget(graph, { x: 2, y: 3 }, signal({ path: 'robot/a' }));
    expect(instance.bindings).toEqual({ series: ['robot/a'] });
    expect([instance.x, instance.y, instance.w, instance.h]).toEqual([2, 3, 4, 2]);
  });
});

describe('storage', () => {
  const board: BoardDoc = { id: 'b1', name: 'Driver', columns: 24, widgets: [] };

  test('round-trips through key/value storage and ignores junk', () => {
    const map = new Map<string, string>();
    const storage: KeyValueStorage = { getItem: (k) => map.get(k) ?? null, setItem: (k, v) => void map.set(k, v) };
    saveBoards([board], storage);
    expect(loadBoards(storage)).toEqual([board]);
    map.set('athena.boards.v1', '{"nope":1}');
    expect(loadBoards(storage)).toEqual([]);
  });

  test('import accepts exported files and single boards, rejects others', () => {
    expect(importBoards(exportBoards([board]))).toEqual([board]);
    expect(importBoards(JSON.stringify(board))).toEqual([board]);
    expect(() => importBoards('{"format":"other"}')).toThrow('Not an Athena board file');
    expect(() => importBoards('not json')).toThrow('Not a JSON file');
  });
});

describe('history', () => {
  test('undo and redo walk snapshots', () => {
    const history = new History<number[]>();
    let state = [1];
    history.push(state);
    state = [1, 2];
    expect(history.canUndo).toBe(true);
    state = history.undo(state)!;
    expect(state).toEqual([1]);
    state = history.redo(state)!;
    expect(state).toEqual([1, 2]);
    expect(history.redo(state)).toBeNull();
  });
});

describe('templates', () => {
  beforeEach(() => {
    clearRegistry();
    for (const type of ['match-timer', 'match-battery', 'match-alerts', 'match-camera', 'match-field', 'mechanismStatus']) registerWidget(def({ type }));
  });

  test('driver board rows end flush with the grid', () => {
    const signals = ['arm', 'turret', 'intake'].map((m, i) =>
      signal({ signalId: i + 1, path: `robot/${m}/State/Reason`, type: 'string', mechanismPath: `robot/${m}`, localPath: 'State/Reason', name: 'Reason' })
    );
    const board = driverBoard(signals);
    const rows = new Map<number, number>();
    for (const w of board.widgets) rows.set(w.y, (rows.get(w.y) ?? 0) + w.w);
    expect([...rows.values()].every((total) => total === board.columns)).toBe(true);
    expect(board.widgets.map((w) => w.type)).toContain('mechanismStatus');
  });
});
