import { describe, expect, test } from 'bun:test';
import { RingBuffer } from './ring';
import { ValueStore } from './store';
import { buildMechanismTree, flattenMechanisms, groupSignals } from './tree';
import { describePath, parsePath } from './path';
import { createSession } from './session';
import { createMockSource } from './mock';
import { MockRobot } from './mock-robot';
import { encodeFrame, encodeDictionary, encodeMarker } from './encoder';
import { decodeRecords } from './decoder';
import { TYPE_TAG, type DecodedRecord, type SignalDescriptor, type SignalValue } from './types';

function descriptor(signalId: number, path: string, extra: Partial<SignalDescriptor> = {}): SignalDescriptor {
  return {
    signalId,
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

function frameOf(sequence: number, ts: number, entries: { signalId: number; typeTag: 1 | 2 | 3; value: SignalValue }[]): DecodedRecord {
  return {
    kind: 'frame',
    frame: {
      version: 2,
      keyframe: false,
      fragmented: false,
      cycleTimestampMicros: ts,
      sequence,
      fragmentIndex: 0,
      fragmentCount: 1,
      entries
    }
  };
}

function frame(sequence: number, ts: number, entries: { signalId: number; value: number }[]): DecodedRecord {
  return {
    kind: 'frame',
    frame: {
      version: 2,
      keyframe: false,
      fragmented: false,
      cycleTimestampMicros: ts,
      sequence,
      fragmentIndex: 0,
      fragmentCount: 1,
      entries: entries.map((e) => ({ ...e, typeTag: TYPE_TAG.f64 }))
    }
  };
}

describe('RingBuffer', () => {
  test('keeps the newest items in order and overwrites the oldest', () => {
    const ring = new RingBuffer<number>(3);
    [1, 2, 3, 4, 5].forEach((n) => ring.push(n));
    expect(ring.toArray()).toEqual([3, 4, 5]);
    expect(ring.latest()).toBe(5);
    expect(ring.size).toBe(3);
  });

  test('rejects a non-positive capacity', () => {
    expect(() => new RingBuffer<number>(0)).toThrow(RangeError);
  });
});

describe('path parsing', () => {
  test('mechanism path stops at the first group, keeping child mechanisms nested', () => {
    expect(parsePath('/Athena/Mechanisms/robot/turret/hood/Values/target')).toEqual({
      mechanismPath: 'robot/turret/hood',
      group: 'Values',
      localPath: 'Values/target',
      name: 'target'
    });
  });

  test('paths without a group fall back to the parent as mechanism', () => {
    expect(parsePath('robot/turret/extra').mechanismPath).toBe('robot/turret');
  });
});

describe('mechanism tree', () => {
  test('parents are created implicitly and children nest under their declaring mechanism', () => {
    const signals = [
      descriptor(1, 'robot/turret/State/state'),
      descriptor(2, 'robot/turret/hood/Values/target'),
      descriptor(3, 'robot/shooter/Devices/motor/current')
    ].map((d) => ({ ...d, ...describePath(d.path) }));
    const roots = buildMechanismTree(signals);
    expect(roots.map((n) => n.path)).toEqual(['robot']);
    const flat = flattenMechanisms(roots).map((n) => n.path);
    expect(flat).toEqual(['robot', 'robot/shooter', 'robot/turret', 'robot/turret/hood']);
    const turret = flattenMechanisms(roots).find((n) => n.path === 'robot/turret');
    expect(turret?.signals.map((s) => s.signalId)).toEqual([1]);
    expect(groupSignals(turret!).State.map((s) => s.signalId)).toEqual([1]);
  });
});

describe('ValueStore', () => {
  test('dictionary, frames, history and sequence gaps', () => {
    const store = new ValueStore({ historyCapacity: 2 });
    store.apply({ kind: 'dictionary', entries: [descriptor(1, 'robot/a/Values/x', { writable: true })] });
    store.apply(frame(1, 100, [{ signalId: 1, value: 1 }]));
    store.apply(frame(4, 200, [{ signalId: 1, value: 2 }, { signalId: 9, value: 0 }]));
    store.apply(frame(5, 300, [{ signalId: 1, value: 3 }]));
    expect(store.latestSample(1)).toEqual({ timestampMicros: 300, value: 3 });
    expect(store.samples(1).map((s) => s.value)).toEqual([2, 3]);
    expect(store.sequenceGaps).toBe(2);
    expect(store.unknownEntries).toBe(1);
    expect(store.descriptorByPath('robot/a/Values/x')?.writable).toBe(true);
  });

  test('markers are capped and metadata merges', () => {
    const store = new ValueStore({ markerCapacity: 2 });
    for (let i = 0; i < 3; i++) {
      store.apply({
        kind: 'marker',
        marker: { timestampMicros: i, kind: 4, label: 'Teleop start', message: String(i), signalId: null }
      });
    }
    expect(store.markers().map((m) => m.message)).toEqual(['1', '2']);
    store.apply({ kind: 'metadata', pairs: { event: 'A' } });
    store.apply({ kind: 'metadata', pairs: { robot: 'R' } });
    expect(store.metadata()).toEqual({ event: 'A', robot: 'R' });
  });

  test('lock state is derived from the role-7 signal at athena/locks/<group>', () => {
    const group = 'ca.frc6390.athena.Demo.InMatch';
    const store = new ValueStore();
    store.apply({
      kind: 'dictionary',
      entries: [
        descriptor(1, 'robot/a/Values/x', { writable: true, lockGroup: group }),
        descriptor(2, `athena/locks/${group}`, { typeTag: TYPE_TAG.bool, type: 'bool', role: 'lock' })
      ]
    });
    expect(store.lockStateOf(1)).toBe('unknown');
    store.apply(frameOf(1, 10, [{ signalId: 2, typeTag: TYPE_TAG.bool, value: true }]));
    expect(store.lockStateOf(1)).toBe('locked');
    store.apply(frameOf(2, 20, [{ signalId: 2, typeTag: TYPE_TAG.bool, value: false }]));
    expect(store.lockStateOf(1)).toBe('open');
    // Read-only and ungrouped signals have no lock state.
    expect(store.lockStateOf(2)).toBe('unknown');
  });

  test('fragmented frames are reassembled by sequence before they reach the history', () => {
    const store = new ValueStore();
    store.apply({ kind: 'dictionary', entries: [descriptor(1, 'robot/a/Values/x'), descriptor(2, 'robot/a/Values/y')] });
    const fragment = (index: number, entries: { signalId: number; typeTag: 3; value: number }[]): DecodedRecord => ({
      kind: 'frame',
      frame: {
        version: 2,
        keyframe: false,
        fragmented: true,
        cycleTimestampMicros: 100,
        sequence: 7,
        fragmentIndex: index,
        fragmentCount: 2,
        entries
      }
    });
    store.apply(fragment(1, [{ signalId: 2, typeTag: 3, value: 2 }]));
    expect(store.latestSample(2)).toBeUndefined();
    store.apply(fragment(0, [{ signalId: 1, typeTag: 3, value: 1 }]));
    expect(store.latestSample(1)?.value).toBe(1);
    expect(store.latestSample(2)?.value).toBe(2);
    expect(store.sequenceGaps).toBe(0);
  });

  test('i64 values beyond 2^53 stay exact as bigints', async () => {
    const big = 9007199254740993n; // 2^53 + 1
    const bytes = new Uint8Array([
      ...encodeDictionary([{ signalId: 1, path: 'robot/a/Values/n', typeTag: TYPE_TAG.i64 }]),
      ...encodeFrame({
        keyframe: true,
        cycleTimestampMicros: 1,
        sequence: 0,
        entries: [
          { signalId: 1, typeTag: TYPE_TAG.i64, value: big },
        ]
      })
    ]);
    const store = new ValueStore();
    store.applyAll(await decodeRecords(bytes));
    expect(store.latestSample(1)?.value).toBe(big);
  });

  test('subscribers are notified and the version advances', () => {
    const store = new ValueStore();
    let calls = 0;
    const off = store.subscribe(() => calls++);
    store.apply({ kind: 'metadata', pairs: {} });
    off();
    store.apply({ kind: 'metadata', pairs: {} });
    expect(calls).toBe(1);
    expect(store.version).toBe(2);
  });
});

describe('end-to-end through the codec', () => {
  test('bytes decoded into the store produce latest values and markers', async () => {
    const bytes = new Uint8Array([
      ...encodeDictionary([{ signalId: 1, path: 'robot/a/State/s', typeTag: TYPE_TAG.string }]),
      ...encodeFrame({
        keyframe: true,
        cycleTimestampMicros: 42,
        sequence: 0,
        entries: [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'ready' }]
      }),
      ...encodeMarker(42, 6, 'disable')
    ]);
    const store = new ValueStore();
    store.applyAll(await decodeRecords(bytes));
    expect(store.latestSample(1)?.value).toBe('ready');
    expect(store.markers()[0].label).toBe('Disable');
  });
});

describe('mock robot', () => {
  test('writes to InMatch signals are rejected while a match is active and accepted otherwise', () => {
    const robot = new MockRobot();
    expect(robot.write('robot/turret/Values/target', 10)).toEqual({ accepted: true, reason: null });
    robot.matchActive = true;
    expect(robot.write('robot/turret/Values/target', 11)).toEqual({
      accepted: false,
      reason: 'rejected: locked (InMatch)'
    });
    robot.step(0);
    expect(robot.value('athena/locks/ca.frc6390.athena.mechanism.persist.Locks$InMatch')).toBe(true);
    // Constraints use the calibration lock and stay writable during a match.
    expect(robot.write('robot/turret/Controls/position/Config/Constraints/MaximumVelocity', 90)).toEqual({
      accepted: true,
      reason: null
    });
  });

  test('type mismatches are reported by the robot', () => {
    const robot = new MockRobot();
    expect(robot.write('robot/turret/Values/target', 'far')).toEqual({
      accepted: false,
      reason: 'rejected: expected number'
    });
  });

  test('the turret tracks its target', () => {
    const robot = new MockRobot();
    robot.write('robot/turret/Values/target', 30);
    for (let i = 0; i < 500; i++) robot.step(0.02);
    expect(Math.abs((robot.value('robot/turret/Controls/position/State/FeedbackPosition') as number) - 30)).toBeLessThan(1);
  });
});

describe('mock source session', () => {
  test('publishes dictionary and live values, and records command replies', async () => {
    const session = createSession(createMockSource({ tickMs: 5 }));
    session.start();
    await new Promise((resolve) => setTimeout(resolve, 150));
    const store = session.store;
    const target = store.descriptorByPath('robot/turret/Values/target');
    expect(target?.writable).toBe(true);
    expect(store.latestSample(target!.signalId)).toBeDefined();
    const reply = await session.commands.write(target!.signalId, 15);
    expect(reply.accepted).toBe(true);
    expect(reply.actualValue).toBe(15);
    expect(store.reply(target!.signalId)?.accepted).toBe(true);
    session.stop();
  });
});
