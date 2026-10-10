import { describe, expect, test } from 'bun:test';
import { copyText, type ClipboardDeps } from './clipboard';
import { isCopyShortcut, widgetMenuItems, widgetMenuPosition, widgetMenuStyle } from './context-menu';
import { widgetKeyAction } from './selection';

describe('copy value', () => {
  test('uses the async clipboard when it works', async () => {
    const written: string[] = [];
    const deps: ClipboardDeps = { writeText: async (t) => void written.push(t), execCopy: () => false };
    expect(await copyText('12.5', deps)).toBe(true);
    expect(written).toEqual(['12.5']);
  });

  test('falls back to the legacy copy path when the async API is refused', async () => {
    const deps: ClipboardDeps = {
      writeText: async () => {
        throw new Error('denied');
      },
      execCopy: (t) => t === '7'
    };
    expect(await copyText('7', deps)).toBe(true);
  });

  test('reports failure when neither path works', async () => {
    expect(await copyText('x', { execCopy: () => false })).toBe(false);
    expect(await copyText('x', {})).toBe(false);
  });
});

describe('widget context menu', () => {
  test('copy is offered only for widgets that show a signal', () => {
    expect(widgetMenuItems({ signalId: null }).map((i) => i.action)).toEqual(['inspector', 'delete']);
    expect(widgetMenuItems({ signalId: 3 }).map((i) => i.action)).toEqual(['inspector', 'copy', 'delete']);
    expect(widgetMenuItems({ signalId: 3 }).at(-1)?.danger).toBe(true);
  });

  test('the menu stays inside the viewport', () => {
    expect(widgetMenuPosition({ x: 2000, y: 2000 }, { width: 1920, height: 1080 })).toEqual({ left: 1700, top: 960 });
    expect(widgetMenuPosition({ x: 0, y: 0 }, { width: 1920, height: 1080 })).toEqual({ left: 8, top: 8 });
    expect(widgetMenuStyle({ x: 30, y: 40 }, { width: 1920, height: 1080 })).toBe('left:30px;top:40px;');
  });

  test('Ctrl/Cmd+C copies only when a menu for a signal-bound widget is open', () => {
    const ctrlC = { key: 'c', ctrlKey: true, metaKey: false, altKey: false };
    expect(isCopyShortcut(ctrlC, { signalId: 1 })).toBe(true);
    expect(isCopyShortcut({ ...ctrlC, key: 'C', metaKey: true, ctrlKey: false }, { signalId: 1 })).toBe(true);
    expect(isCopyShortcut(ctrlC, { signalId: null })).toBe(false);
    expect(isCopyShortcut(ctrlC, null)).toBe(false);
    expect(isCopyShortcut({ ...ctrlC, altKey: true }, { signalId: 1 })).toBe(false);
  });
});

describe('keyboard on a widget card', () => {
  test('Enter and Space select, Delete and Backspace remove, I opens the inspector', () => {
    expect(widgetKeyAction('Enter')).toBe('select');
    expect(widgetKeyAction(' ')).toBe('select');
    expect(widgetKeyAction('Delete')).toBe('remove');
    expect(widgetKeyAction('Backspace')).toBe('remove');
    expect(widgetKeyAction('i')).toBe('inspector');
    expect(widgetKeyAction('I')).toBe('inspector');
    expect(widgetKeyAction('x')).toBeNull();
  });
});
