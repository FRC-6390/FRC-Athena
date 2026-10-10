/**
 * Local storage for named canvas dashboards in the new shell.
 *
 * Each layout is stored under its own key (`<prefix><name>`) in the same JSON shape the classic canvas
 * exports (`serializeLayoutExport`), so files move freely between the classic UI and the new shell. An
 * index key keeps the user's tab order.
 */
import {
  createDefaultLayout,
  parseLayoutImport,
  serializeLayoutExport,
  type DashboardLayoutState
} from './dashboard';

export const LAYOUT_KEY_PREFIX = 'arcp.canvas.layout.v1:';
export const LAYOUT_INDEX_KEY = 'arcp.canvas.layouts.index.v1';
export const LAYOUT_NAME_MAX = 40;

/** The subset of the Web Storage API the store needs. Tests pass an in-memory map. */
export type KeyValueStorage = {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
};

export type LayoutStore = {
  /** Layout names in the user's order. */
  list(): string[];
  load(name: string): DashboardLayoutState | null;
  /** Writes the layout under `name`, adding it to the index when new. */
  save(name: string, layout: DashboardLayoutState): void;
  /** Creates an empty dashboard with a unique name derived from `name`. Returns the stored name. */
  create(name: string): string;
  /** Renames a layout. Returns false when the source is missing or the target name is taken. */
  rename(from: string, to: string): boolean;
  /** Copies a layout under a unique name. Returns the new name, or null when the source is missing. */
  duplicate(name: string): string | null;
  remove(name: string): void;
  /** Layout JSON for export, or null when the layout is missing. */
  exportText(name: string): string | null;
  /** Imports layout JSON as a new layout. Throws on invalid input. Returns the stored name. */
  importText(text: string, preferredName: string): string;
};

/** Trims, collapses whitespace and limits the length of a user-entered dashboard name. */
export function cleanLayoutName(raw: string): string {
  return raw.replace(/\s+/g, ' ').trim().slice(0, LAYOUT_NAME_MAX);
}

/** `base`, or `base 2`, `base 3`, ... so the result is not in `taken`. Empty bases become "Dashboard". */
export function uniqueLayoutName(base: string, taken: readonly string[]): string {
  const root = cleanLayoutName(base) || 'Dashboard';
  if (!taken.includes(root)) return root;
  for (let index = 2; ; index++) {
    const suffix = ` ${index}`;
    const candidate = `${root.slice(0, LAYOUT_NAME_MAX - suffix.length)}${suffix}`;
    if (!taken.includes(candidate)) return candidate;
  }
}

/** The browser's localStorage, or null when it is unavailable (private mode, SSR, tests without a DOM). */
export function browserLayoutStorage(): KeyValueStorage | null {
  try {
    return typeof localStorage === 'undefined' ? null : localStorage;
  } catch {
    return null;
  }
}

function readIndex(storage: KeyValueStorage): string[] {
  try {
    const raw = storage.getItem(LAYOUT_INDEX_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw) as unknown;
    return Array.isArray(parsed) ? parsed.filter((n): n is string => typeof n === 'string') : [];
  } catch {
    return [];
  }
}

function writeIndex(storage: KeyValueStorage, names: string[]): void {
  storage.setItem(LAYOUT_INDEX_KEY, JSON.stringify(names));
}

/**
 * Creates a layout store over `storage`. Every write is best-effort: a failing storage keeps the UI usable
 * and the layout simply is not persisted.
 */
export function createLayoutStore(storage: KeyValueStorage | null): LayoutStore {
  const mem = new Map<string, string>();
  const backing: KeyValueStorage = storage ?? {
    getItem: (key) => mem.get(key) ?? null,
    setItem: (key, value) => void mem.set(key, value),
    removeItem: (key) => void mem.delete(key)
  };

  const keyFor = (name: string) => `${LAYOUT_KEY_PREFIX}${name}`;

  const list = (): string[] => readIndex(backing);

  const readLayout = (name: string): DashboardLayoutState | null => {
    try {
      const raw = backing.getItem(keyFor(name));
      if (!raw) return null;
      return parseLayoutImport(raw);
    } catch {
      return null;
    }
  };

  const save = (name: string, layout: DashboardLayoutState): void => {
    try {
      backing.setItem(keyFor(name), serializeLayoutExport(layout));
      const names = list();
      if (!names.includes(name)) writeIndex(backing, [...names, name]);
    } catch {
      // Best-effort persistence; the canvas keeps working in memory.
    }
  };

  return {
    list,
    load: readLayout,
    save,
    create(name) {
      const stored = uniqueLayoutName(name, list());
      save(stored, createDefaultLayout());
      return stored;
    },
    rename(from, to) {
      const target = cleanLayoutName(to);
      const names = list();
      if (!target || !names.includes(from)) return false;
      if (target === from) return true;
      if (names.includes(target)) return false;
      const layout = readLayout(from);
      if (!layout) return false;
      save(target, layout);
      backing.removeItem(keyFor(from));
      writeIndex(
        backing,
        names.map((n) => (n === from ? target : n)).filter((n, i, all) => all.indexOf(n) === i)
      );
      return true;
    },
    duplicate(name) {
      const layout = readLayout(name);
      if (!layout) return null;
      const stored = uniqueLayoutName(`${name} copy`, list());
      save(stored, layout);
      return stored;
    },
    remove(name) {
      backing.removeItem(keyFor(name));
      writeIndex(
        backing,
        list().filter((n) => n !== name)
      );
    },
    exportText(name) {
      const layout = readLayout(name);
      return layout ? serializeLayoutExport(layout) : null;
    },
    importText(text, preferredName) {
      const layout = parseLayoutImport(text);
      const stored = uniqueLayoutName(preferredName, list());
      save(stored, layout);
      return stored;
    }
  };
}
