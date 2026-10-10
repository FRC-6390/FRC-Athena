import type { BoardDoc } from './types';

/** Boards are a per-viewer convenience kept in localStorage, plus JSON import/export for sharing. */
const KEY = 'athena.boards.v1';
const FORMAT = 'athena-board';
const VERSION = 1;

export type BoardFile = { format: typeof FORMAT; version: number; boards: BoardDoc[] };

export interface KeyValueStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

function defaultStorage(): KeyValueStorage | null {
  try {
    return typeof localStorage === 'undefined' ? null : localStorage;
  } catch {
    return null;
  }
}

export function isBoard(value: unknown): value is BoardDoc {
  if (!value || typeof value !== 'object') return false;
  const b = value as Partial<BoardDoc>;
  return typeof b.id === 'string' && typeof b.name === 'string' && typeof b.columns === 'number' && Array.isArray(b.widgets);
}

export function loadBoards(storage: KeyValueStorage | null = defaultStorage()): BoardDoc[] {
  if (!storage) return [];
  try {
    const raw = storage.getItem(KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw) as unknown;
    return Array.isArray(parsed) ? parsed.filter(isBoard) : [];
  } catch {
    return [];
  }
}

export function saveBoards(boards: BoardDoc[], storage: KeyValueStorage | null = defaultStorage()): void {
  if (!storage) return;
  try {
    storage.setItem(KEY, JSON.stringify(boards));
  } catch {
    // Storage full or blocked: boards stay in memory for this session.
  }
}

export function exportBoards(boards: BoardDoc[]): string {
  const file: BoardFile = { format: FORMAT, version: VERSION, boards };
  return JSON.stringify(file, null, 2);
}

/** Parses an exported file (or a single board). Throws a readable error for anything else. */
export function importBoards(text: string): BoardDoc[] {
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error('Not a JSON file');
  }
  if (isBoard(parsed)) return [parsed];
  const file = parsed as Partial<BoardFile>;
  if (file && file.format === FORMAT && Array.isArray(file.boards)) {
    const boards = file.boards.filter(isBoard);
    if (boards.length > 0) return boards;
  }
  throw new Error('Not an Athena board file');
}
