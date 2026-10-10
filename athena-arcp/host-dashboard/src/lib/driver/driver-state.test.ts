import { describe, expect, test } from 'bun:test';
import { ValueStore } from '../data/store';
import { deriveDriverState, formatClock, matchPhase } from './driver-state';
import { describePath } from '../data/path';
import { MARKER_KIND, TYPE_TAG, type DecodedRecord, type SignalDescriptor } from '../data/types';

function desc(id: number, path: string, extra: Partial<SignalDescriptor> = {}): SignalDescriptor {
  return {
    signalId: id,
    ...describePath(path),
    type: 'f64',
    typeTag: TYPE_TAG.f64,
    unit: '',
    role: 'value',
    writable: false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0,
    ...extra
  };
}

function frame(ts: number, entries: { signalId: number; typeTag: 1 | 2 | 3 | 4; value: number | string | boolean }[]): DecodedRecord {
  return {
    kind: 'frame',
    frame: {
      version: 2,
      keyframe: true,
      fragmented: false,
      cycleTimestampMicros: ts,
      sequence: ts,
      fragmentIndex: 0,
      fragmentCount: 1,
      entries: entries as never
    }
  };
}

function marker(ts: number, kind: number, message = '') {
  return { kind: 'marker' as const, marker: { timestampMicros: ts, kind, label: '', message, signalId: null } };
}

describe('driver state', () => {
  test('match phase follows the latest match marker', () => {
    const store = new ValueStore();
    expect(matchPhase(store).phase).toBe('idle');
    store.apply(marker(1_000_000, MARKER_KIND.AUTO_START));
    expect(matchPhase(store)).toEqual({ phase: 'auto', since: 1_000_000 });
    store.apply(marker(16_000_000, MARKER_KIND.TELEOP_START));
    expect(matchPhase(store).phase).toBe('teleop');
    store.apply(marker(60_000_000, MARKER_KIND.MATCH_END));
    expect(matchPhase(store).phase).toBe('ended');
  });

  test('teleop countdown uses robot time from the newest sample', () => {
    const store = new ValueStore();
    store.apply({ kind: 'dictionary', entries: [desc(1, 'robot/a/Values/x')] });
    store.apply(frame(20_000_000, [{ signalId: 1, typeTag: TYPE_TAG.f64, value: 1 }]));
    store.apply(marker(20_000_000, MARKER_KIND.TELEOP_START));
    store.apply(frame(50_000_000, [{ signalId: 1, typeTag: TYPE_TAG.f64, value: 2 }]));
    const state = deriveDriverState(store);
    expect(state.match.phase).toBe('teleop');
    expect(state.match.elapsedSeconds).toBe(30);
    expect(state.match.remainingSeconds).toBe(105);
    expect(formatClock(state.match.remainingSeconds)).toBe('1:45');
  });

  test('faults and non-normal stages raise alerts; normal stages do not', () => {
    const store = new ValueStore();
    store.apply({
      kind: 'dictionary',
      entries: [
        desc(1, 'robot/turret/State/stage', { type: 'string', typeTag: TYPE_TAG.string, role: 'state', name: 'stage' }),
        desc(2, 'robot/shooter/State/stage', { type: 'string', typeTag: TYPE_TAG.string, role: 'state' })
      ]
    });
    store.apply(frame(30_000_000, [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'HOT' }, { signalId: 2, typeTag: TYPE_TAG.string, value: 'NORMAL' }]));
    store.apply(marker(25_000_000, MARKER_KIND.FAULT, 'brownout'));
    store.apply(marker(1_000_000, MARKER_KIND.FAULT, 'old fault'));
    const state = deriveDriverState(store);
    expect(state.alerts.map((a) => a.text)).toEqual(['brownout', 'robot/turret: HOT']);
  });

  test('power split and mode come from the athena/power and athena/mode signals', () => {
    const store = new ValueStore();
    store.apply({
      kind: 'dictionary',
      entries: [
        desc(1, 'athena/power/battery/voltage'),
        desc(2, 'athena/power/turret/current'),
        desc(3, 'athena/power/drive/current'),
        desc(4, 'athena/mode/operator', { type: 'string', typeTag: TYPE_TAG.string })
      ]
    });
    store.apply(
      frame(1_000_000, [
        { signalId: 1, typeTag: TYPE_TAG.f64, value: 12.1 },
        { signalId: 2, typeTag: TYPE_TAG.f64, value: 10 },
        { signalId: 3, typeTag: TYPE_TAG.f64, value: 30 },
        { signalId: 4, typeTag: TYPE_TAG.string, value: 'Teleop' }
      ])
    );
    const state = deriveDriverState(store);
    expect(state.battery.volts).toBe(12.1);
    expect(state.power.totalAmps).toBe(40);
    expect(state.power.consumers.map((c) => [c.name, c.share])).toEqual([
      ['drive', 0.75],
      ['turret', 0.25]
    ]);
    expect(state.mode).toBe('Teleop');
  });

  test('formatClock pads seconds and handles idle', () => {
    expect(formatClock(null)).toBe('--:--');
    expect(formatClock(9.2)).toBe('0:10');
    expect(formatClock(135)).toBe('2:15');
  });
});
