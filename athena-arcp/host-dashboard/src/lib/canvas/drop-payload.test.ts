import { describe, expect, test } from 'bun:test';
import { LAYOUT_DRAG_TYPE, SIGNAL_DRAG_TYPE, TOPIC_DRAG_TYPE, acceptsDragTypes, parseDropPayload } from './drop-payload';

function transfer(data: Record<string, string>): (type: string) => string {
  return (type) => data[type] ?? '';
}

describe('drop payloads', () => {
  test('typed payloads are read first', () => {
    expect(parseDropPayload(transfer({ [SIGNAL_DRAG_TYPE]: '42' }))).toEqual({ kind: 'signal', signalId: 42 });
    expect(parseDropPayload(transfer({ [TOPIC_DRAG_TYPE]: 'robot/turret' }))).toEqual({
      kind: 'topic',
      topicPath: 'robot/turret'
    });
    expect(parseDropPayload(transfer({ [LAYOUT_DRAG_TYPE]: 'layout_grid' }))).toEqual({
      kind: 'layout',
      layoutKind: 'layout_grid'
    });
  });

  test('plain text falls back to prefixed topics and layout tools, then to a bare signal id', () => {
    expect(parseDropPayload(transfer({ 'text/plain': 'topic:robot/drive' }))).toEqual({
      kind: 'topic',
      topicPath: 'robot/drive'
    });
    expect(parseDropPayload(transfer({ 'text/plain': 'layout:layout_title' }))).toEqual({
      kind: 'layout',
      layoutKind: 'layout_title'
    });
    expect(parseDropPayload(transfer({ 'text/plain': ' 7 ' }))).toEqual({ kind: 'signal', signalId: 7 });
  });

  test('anything unusable is rejected', () => {
    expect(parseDropPayload(transfer({ 'text/plain': 'hello' }))).toBeNull();
    expect(parseDropPayload(transfer({ 'text/plain': '0' }))).toBeNull();
    expect(parseDropPayload(transfer({}))).toBeNull();
  });

  test('only canvas drag types are accepted', () => {
    expect(acceptsDragTypes([SIGNAL_DRAG_TYPE])).toBe(true);
    expect(acceptsDragTypes(['text/plain'])).toBe(true);
    expect(acceptsDragTypes(['Files'])).toBe(false);
  });
});
