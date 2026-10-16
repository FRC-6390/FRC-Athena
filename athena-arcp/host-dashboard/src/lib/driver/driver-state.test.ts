import { describe, expect, test } from 'bun:test';
import { ValueStore } from '../data/store';
import { deriveDriverState, formatClock, matchPhase, phaseFromMode } from './driver-state';
import { describePath } from '../data/path';
import { MARKER_KIND, ROBOT_PATHS, TYPE_TAG, type DecodedRecord, type SignalDescriptor } from '../data/types';
import { descriptorFor, realDictionary } from '../data/fixtures/real-dictionary';

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

/** A store holding the real dictionary, with the given values written as one frame at `ts`. */
function storeWithReal(values: Record<string, number | string>, ts = 1_000_000): ValueStore {
  const store = new ValueStore();
  const dictionary = realDictionary();
  const extra = Object.keys(values)
    .filter((path) => !dictionary.some((d) => d.path === path))
    .map((path, index) =>
      descriptorFor(10_000 + index, path, typeof values[path] === 'string' ? 'string' : 'f64', 'system')
    );
  store.apply({ kind: 'dictionary', entries: [...dictionary, ...extra] });
  const entries = Object.entries(values).map(([path, value]) => {
    const signal = store.descriptorByPath(path)!;
    return { signalId: signal.signalId, typeTag: (typeof value === 'string' ? TYPE_TAG.string : TYPE_TAG.f64) as 3 | 4, value };
  });
  store.apply(frame(ts, entries));
  return store;
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

  test('the robot match clock wins over the marker countdown when it is published', () => {
    const store = storeWithReal({ [ROBOT_PATHS.mode]: 'Teleop', [ROBOT_PATHS.matchTime]: 97 }, 50_000_000);
    store.apply(marker(20_000_000, MARKER_KIND.TELEOP_START));
    const state = deriveDriverState(store);
    expect(state.match.phase).toBe('teleop');
    expect(state.match.remainingSeconds).toBe(97);
  });

  test('without match markers the DriverStation mode gives the phase; Disabled is idle', () => {
    expect(phaseFromMode('Autonomous')).toBe('auto');
    expect(phaseFromMode('Teleop')).toBe('teleop');
    expect(phaseFromMode('Disabled')).toBe('idle');
    expect(phaseFromMode(null)).toBe('idle');
    const store = storeWithReal({ [ROBOT_PATHS.mode]: 'Autonomous', [ROBOT_PATHS.matchTime]: 11 });
    expect(deriveDriverState(store).match.phase).toBe('auto');
    expect(deriveDriverState(store).match.remainingSeconds).toBe(11);
  });

  test('mechanism faults and stage-change markers raise alerts; empty fault text does not', () => {
    const store = new ValueStore();
    store.apply({
      kind: 'dictionary',
      entries: [
        desc(1, 'robot/turret/State/Fault', { type: 'string', typeTag: TYPE_TAG.string, role: 'state' }),
        desc(2, 'robot/shooter/State/Fault', { type: 'string', typeTag: TYPE_TAG.string, role: 'state' })
      ]
    });
    store.apply(frame(30_000_000, [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'motor over temperature' }, { signalId: 2, typeTag: TYPE_TAG.string, value: '' }]));
    store.apply(marker(25_000_000, MARKER_KIND.FAULT, 'brownout'));
    store.apply(marker(26_000_000, MARKER_KIND.STAGE_CHANGE, 'turret: stage 2'));
    store.apply(marker(1_000_000, MARKER_KIND.FAULT, 'old fault'));
    const state = deriveDriverState(store);
    expect(state.alerts.map((a) => [a.level, a.text])).toEqual([
      ['fault', 'brownout'],
      ['warn', 'turret: stage 2'],
      ['fault', 'robot/turret: motor over temperature']
    ]);
  });

  test('power split uses published athena/power currents, else each motor supply current', () => {
    const real = storeWithReal({
      [ROBOT_PATHS.batteryVolts]: 12.1,
      'robot/turret/Devices/motor/State/SupplyCurrentAmps': 10,
      'robot/shooter/Devices/motor/State/SupplyCurrentAmps': 30,
      'robot/shooter/Devices/motor/State/StatorCurrentAmps': 99
    });
    const fromMechanisms = deriveDriverState(real);
    expect(fromMechanisms.battery.volts).toBe(12.1);
    expect(fromMechanisms.power.totalAmps).toBe(40);
    // Every motor in the real dictionary is a consumer; the idle ones have zero amps.
    expect(fromMechanisms.power.consumers.length).toBeGreaterThan(2);
    const active = fromMechanisms.power.consumers.filter((c) => c.amps > 0);
    expect(active.map((c) => [c.name, c.share])).toEqual([
      ['shooter/motor', 0.75],
      ['turret/motor', 0.25]
    ]);

    const published = new ValueStore();
    published.apply({
      kind: 'dictionary',
      entries: [
        desc(1, 'athena/power/battery/voltage'),
        desc(2, 'athena/power/turret/current'),
        desc(3, 'athena/power/drive/current'),
        desc(4, 'robot/turret/Devices/motor/State/SupplyCurrentAmps')
      ]
    });
    published.apply(
      frame(1_000_000, [
        { signalId: 1, typeTag: TYPE_TAG.f64, value: 12.1 },
        { signalId: 2, typeTag: TYPE_TAG.f64, value: 10 },
        { signalId: 3, typeTag: TYPE_TAG.f64, value: 30 },
        { signalId: 4, typeTag: TYPE_TAG.f64, value: 99 }
      ])
    );
    const state = deriveDriverState(published);
    expect(state.power.consumers.map((c) => c.name)).toEqual(['drive', 'turret']);
    expect(state.power.totalAmps).toBe(40);
  });

  test('mode and alliance come from the robot system signals and are null before they arrive', () => {
    expect(deriveDriverState(new ValueStore()).mode).toBeNull();
    const state = deriveDriverState(storeWithReal({ [ROBOT_PATHS.mode]: 'Teleop', [ROBOT_PATHS.alliance]: 'Red' }));
    expect(state.mode).toBe('Teleop');
    expect(state.alliance).toBe('Red');
    expect(state.battery.volts).toBeNull();
  });

  test('the real dictionary has no stage signal, so stage text never raises an alert', () => {
    expect(realDictionary().some((s) => s.localPath.endsWith('State/stage'))).toBe(false);
    expect(deriveDriverState(storeWithReal({})).alerts).toEqual([]);
  });

  test('a camera stuck UNSYNCED for over 2 s warns only while the robot is enabled', () => {
    const build = (mode: string) => {
      const store = new ValueStore();
      store.apply({
        kind: 'dictionary',
        entries: [
          desc(1, 'athena/vision/front/State', { type: 'string', typeTag: TYPE_TAG.string, role: 'value' }),
          desc(2, ROBOT_PATHS.mode, { type: 'string', typeTag: TYPE_TAG.string, role: 'system' })
        ]
      });
      store.apply(frame(1_000_000, [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'UNSYNCED' }, { signalId: 2, typeTag: TYPE_TAG.string, value: mode }]));
      store.apply(frame(5_000_000, [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'UNSYNCED' }, { signalId: 2, typeTag: TYPE_TAG.string, value: mode }]));
      return deriveDriverState(store).alerts.map((a) => [a.level, a.text]);
    };
    expect(build('Teleop')).toEqual([['warn', 'front camera: UNSYNCED']]);
    expect(build('Autonomous')).toEqual([['warn', 'front camera: UNSYNCED']]);
    expect(build('Disabled')).toEqual([]);
  });

  test('formatClock pads seconds and handles idle', () => {
    expect(formatClock(null)).toBe('--:--');
    expect(formatClock(9.2)).toBe('0:10');
    expect(formatClock(135)).toBe('2:15');
  });
});
