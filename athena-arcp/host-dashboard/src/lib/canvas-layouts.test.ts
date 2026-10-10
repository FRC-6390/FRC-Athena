import { describe, expect, test } from 'bun:test';
import { createDefaultLayout, type DashboardLayoutState, type DashboardWidget } from './dashboard';
import {
  cleanLayoutName,
  createLayoutStore,
  LAYOUT_INDEX_KEY,
  LAYOUT_KEY_PREFIX,
  LAYOUT_NAME_MAX,
  uniqueLayoutName,
  type KeyValueStorage
} from './canvas-layouts';

function memoryStorage(): KeyValueStorage & { data: Map<string, string> } {
  const data = new Map<string, string>();
  return {
    data,
    getItem: (key) => data.get(key) ?? null,
    setItem: (key, value) => void data.set(key, value),
    removeItem: (key) => void data.delete(key)
  };
}

function layoutWithWidget(title: string): DashboardLayoutState {
  const base = createDefaultLayout();
  const widget: DashboardWidget = {
    id: `w-${title}`,
    signalId: 7,
    kind: 'metric',
    title,
    layout: { x: 1, y: 1, w: 4, h: 3 }
  };
  return { ...base, tabs: [{ ...base.tabs[0], widgets: [widget] }] };
}

describe('name helpers', () => {
  test('cleanLayoutName trims, collapses spaces and limits length', () => {
    expect(cleanLayoutName('  Turret   debug ')).toBe('Turret debug');
    expect(cleanLayoutName('x'.repeat(80))).toHaveLength(LAYOUT_NAME_MAX);
  });

  test('uniqueLayoutName appends a counter instead of colliding', () => {
    expect(uniqueLayoutName('Pit', [])).toBe('Pit');
    expect(uniqueLayoutName('Pit', ['Pit'])).toBe('Pit 2');
    expect(uniqueLayoutName('Pit', ['Pit', 'Pit 2'])).toBe('Pit 3');
    expect(uniqueLayoutName('   ', [])).toBe('Dashboard');
  });
});

describe('createLayoutStore', () => {
  test('save and load round-trip through the classic export format', () => {
    const storage = memoryStorage();
    const store = createLayoutStore(storage);
    const layout = layoutWithWidget('Shooter');
    store.save('Match', layout);

    expect(store.list()).toEqual(['Match']);
    expect(store.load('Match')).toEqual(layout);
    const stored = JSON.parse(storage.data.get(`${LAYOUT_KEY_PREFIX}Match`) ?? '{}') as { format: string; layout: unknown };
    expect(stored.layout).toEqual(layout);
    expect(typeof stored.format).toBe('string');
  });

  test('re-saving keeps the index order and does not duplicate names', () => {
    const store = createLayoutStore(memoryStorage());
    store.save('A', createDefaultLayout());
    store.save('B', createDefaultLayout());
    store.save('A', layoutWithWidget('again'));
    expect(store.list()).toEqual(['A', 'B']);
  });

  test('create picks unique names and starts with a default page', () => {
    const store = createLayoutStore(memoryStorage());
    expect(store.create('Pit')).toBe('Pit');
    expect(store.create('Pit')).toBe('Pit 2');
    expect(store.list()).toEqual(['Pit', 'Pit 2']);
    expect(store.load('Pit 2')?.tabs).toHaveLength(1);
  });

  test('rename moves the layout and keeps its position', () => {
    const store = createLayoutStore(memoryStorage());
    store.save('A', layoutWithWidget('a'));
    store.save('B', createDefaultLayout());
    expect(store.rename('A', 'Alpha')).toBe(true);
    expect(store.list()).toEqual(['Alpha', 'B']);
    expect(store.load('A')).toBeNull();
    expect(store.load('Alpha')?.tabs[0].widgets[0].title).toBe('a');
  });

  test('rename refuses taken or missing names and is a no-op for the same name', () => {
    const store = createLayoutStore(memoryStorage());
    store.save('A', createDefaultLayout());
    store.save('B', createDefaultLayout());
    expect(store.rename('A', 'B')).toBe(false);
    expect(store.rename('Z', 'Y')).toBe(false);
    expect(store.rename('A', '  ')).toBe(false);
    expect(store.rename('A', 'A')).toBe(true);
    expect(store.load('A')).not.toBeNull();
    expect(store.list()).toEqual(['A', 'B']);
  });

  test('duplicate copies content under a new unique name', () => {
    const store = createLayoutStore(memoryStorage());
    store.save('Main', layoutWithWidget('m'));
    const copy = store.duplicate('Main');
    expect(copy).toBe('Main copy');
    expect(store.load('Main copy')).toEqual(store.load('Main'));
    expect(store.duplicate('Missing')).toBeNull();
  });

  test('remove deletes the key and the index entry', () => {
    const storage = memoryStorage();
    const store = createLayoutStore(storage);
    store.save('A', createDefaultLayout());
    store.save('B', createDefaultLayout());
    store.remove('A');
    expect(store.list()).toEqual(['B']);
    expect(storage.data.has(`${LAYOUT_KEY_PREFIX}A`)).toBe(false);
  });

  test('import stores the file as a new layout and export reproduces it', () => {
    const store = createLayoutStore(memoryStorage());
    store.save('Src', layoutWithWidget('x'));
    const text = store.exportText('Src');
    expect(text).not.toBeNull();
    const name = store.importText(text ?? '', 'Src');
    expect(name).toBe('Src 2');
    expect(store.load(name)).toEqual(store.load('Src'));
    expect(() => store.importText('{"nope":1}', 'Bad')).toThrow();
    expect(store.list()).toEqual(['Src', 'Src 2']);
  });

  test('a corrupt index or layout is ignored instead of throwing', () => {
    const storage = memoryStorage();
    storage.setItem(LAYOUT_INDEX_KEY, '{not json');
    storage.setItem(`${LAYOUT_KEY_PREFIX}Bad`, 'garbage');
    const store = createLayoutStore(storage);
    expect(store.list()).toEqual([]);
    expect(store.load('Bad')).toBeNull();
  });

  test('works without any storage by falling back to memory', () => {
    const store = createLayoutStore(null);
    store.save('Mem', createDefaultLayout());
    expect(store.list()).toEqual(['Mem']);
  });
});
