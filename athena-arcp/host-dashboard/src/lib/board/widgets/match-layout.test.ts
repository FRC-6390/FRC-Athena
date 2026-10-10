import { describe, expect, test } from 'bun:test';
import { parseInline, parseItems, parseNote, safeImageUrl, unescapeLines } from './match-layout';
import { checklistKey, hashText, loadChecked, saveChecked } from './match-checklist';
import { STOPPED, elapsedMs, formatTenths, remainingMs, reset, start, stop } from './match-stopwatch';
import { logLines } from './match-log';
import { chooserOptions, parseOptions, selectionState } from './match-chooser';
import { retryDelayMs, streamSource, streamUrlProblem } from './match-camera';

describe('layout text', () => {
  test('escapes and note blocks', () => {
    expect(unescapeLines('a\\nb')).toBe('a\nb');
    expect(parseNote('# Pit\\n- one\\n- **two**\\n\\nSome text\\ncontinued')).toEqual([
      { kind: 'heading', level: 1, text: 'Pit' },
      { kind: 'list', items: ['one', '**two**'] },
      { kind: 'paragraph', text: 'Some text continued' }
    ]);
  });

  test('inline bold keeps unclosed markers as text', () => {
    expect(parseInline('a **b** c')).toEqual([
      { text: 'a ', bold: false },
      { text: 'b', bold: true },
      { text: ' c', bold: false }
    ]);
    expect(parseInline('a **b')).toEqual([{ text: 'a ', bold: false }, { text: '**b', bold: false }]);
  });

  test('items and image urls', () => {
    expect(parseItems(' bolt | battery \\n |  ')).toEqual(['bolt', 'battery']);
    expect(safeImageUrl('https://x.test/a.png')).toBe('https://x.test/a.png');
    expect(safeImageUrl('/static/logo.png')).toBe('/static/logo.png');
    expect(safeImageUrl('data:image/png;base64,AAAA')).not.toBeNull();
    expect(safeImageUrl('javascript:alert(1)')).toBeNull();
    expect(safeImageUrl('//evil.test/x.png')).toBeNull();
    expect(safeImageUrl('  ')).toBeNull();
  });
});

describe('checklist memory', () => {
  test('hash and key', () => {
    expect(hashText('abc')).toBe(hashText('abc'));
    expect(hashText('abc')).not.toBe(hashText('abd'));
    expect(checklistKey('  pit-1 ', ['x'])).toBe('athena.board.checklist.pit-1');
    expect(checklistKey('', ['x'])).toBe(`athena.board.checklist.${hashText('x')}`);
  });

  test('round-trips through localStorage and tolerates storage errors', () => {
    const store = new Map<string, string>();
    (globalThis as { localStorage?: unknown }).localStorage = {
      getItem: (k: string) => store.get(k) ?? null,
      setItem: (k: string, v: string) => store.set(k, v),
      removeItem: (k: string) => store.delete(k)
    };
    saveChecked('k', ['bolt']);
    expect(loadChecked('k')).toEqual(['bolt']);
    saveChecked('k', []);
    expect(loadChecked('k')).toEqual([]);
    (globalThis as { localStorage?: unknown }).localStorage = {
      getItem: () => {
        throw new Error('blocked');
      },
      setItem: () => {
        throw new Error('blocked');
      },
      removeItem: () => {
        throw new Error('blocked');
      }
    };
    expect(loadChecked('k')).toEqual([]);
    expect(() => saveChecked('k', ['a'])).not.toThrow();
    delete (globalThis as { localStorage?: unknown }).localStorage;
  });
});

describe('stopwatch', () => {
  test('start, stop, resume and reset', () => {
    let sw = start(STOPPED, 1000);
    expect(elapsedMs(sw, 1500)).toBe(500);
    sw = stop(sw, 2000);
    expect(elapsedMs(sw, 9000)).toBe(1000);
    sw = start(sw, 10_000);
    expect(elapsedMs(sw, 10_250)).toBe(1250);
    expect(elapsedMs(reset(), 99)).toBe(0);
  });

  test('countdown never goes below zero and formats tenths', () => {
    const sw = start(STOPPED, 0);
    expect(remainingMs(10, sw, 4000)).toBe(6000);
    expect(remainingMs(10, sw, 99_000)).toBe(0);
    expect(formatTenths(75_400)).toBe('1:15.4');
    expect(formatTenths(-5)).toBe('0:00.0');
  });
});

describe('log feed', () => {
  test('collapses repeats, newest first, capped, optional clear', () => {
    const s = (t: number, v: string) => ({ timestampMicros: t, value: v });
    const lines = logLines([s(1, 'a'), s(2, 'a'), s(3, ''), s(4, 'b'), s(5, 'a')], 10);
    expect(lines).toEqual([
      { timestampMicros: 5, text: 'a' },
      { timestampMicros: 4, text: 'b' },
      { timestampMicros: 1, text: 'a' }
    ]);
    expect(logLines([s(1, 'a'), s(2, 'b'), s(3, 'c')], 2).map((l) => l.text)).toEqual(['c', 'b']);
    expect(logLines([s(1, 'a'), s(2, 'b')], 10, 1).map((l) => l.text)).toEqual(['b']);
  });
});

describe('chooser and camera', () => {
  test('options and selection', () => {
    expect(parseOptions('Left | Right|Left| ')).toEqual(['Left', 'Right']);
    expect(chooserOptions(['A', 'A', 'B'], 'x|y')).toEqual(['A', 'B']);
    expect(chooserOptions([], 'x|y')).toEqual(['x', 'y']);
    expect(selectionState(['A', 'B'], 'B')).toEqual({ selected: 'B', unknownValue: null });
    expect(selectionState(['A'], 'Z')).toEqual({ selected: null, unknownValue: 'Z' });
    expect(selectionState(['A'], undefined)).toEqual({ selected: null, unknownValue: null });
  });

  test('stream url checks, retry timing and cache busting', () => {
    expect(streamUrlProblem('')).not.toBeNull();
    expect(streamUrlProblem('ftp://x')).not.toBeNull();
    expect(streamUrlProblem('not a url')).not.toBeNull();
    expect(streamUrlProblem('http://photonvision.local:1182/stream.mjpg')).toBeNull();
    expect([0, 1, 2, 3, 9].map(retryDelayMs)).toEqual([1000, 2000, 4000, 5000, 5000]);
    expect(streamSource('http://x/s.mjpg', 0)).toBe('http://x/s.mjpg');
    expect(streamSource('http://x/s.mjpg', 2)).toBe('http://x/s.mjpg?retry=2');
    expect(streamSource('http://x/s.mjpg?a=1', 2)).toBe('http://x/s.mjpg?a=1&retry=2');
  });
});
