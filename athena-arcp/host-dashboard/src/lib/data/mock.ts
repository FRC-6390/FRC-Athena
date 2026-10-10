/**
 * Mock data source: a simulated turret and shooter that runs without a robot. Frames are encoded with
 * the E1-W encoder and decoded again, so the mock exercises the same byte path as a real stream.
 */
import { encodeDictionary, encodeFrame, encodeMarker, encodeMetadata, concat, type FrameInput } from './encoder';
import { decodeRecords } from './decoder';
import { MARKER_KIND, type LockState, type SignalValue } from './types';
import { MOCK_SPECS, MockRobot, type MockSpec } from './mock-robot';
import type { CommandReply, RobotCommands } from './commands';
import type { DataSource, SourceSink } from './source';
import { MECHANISM_GROUPS } from './path';

export const MOCK_TICK_MS = 20;
const KEYFRAME_EVERY = 50;

export type MockSourceOptions = {
  tickMs?: number;
  /** Override the signal list (tests). */
  specs?: MockSpec[];
};

export function createMockSource(options: MockSourceOptions = {}): DataSource {
  const tickMs = options.tickMs ?? MOCK_TICK_MS;
  const robot = new MockRobot(options.specs ?? MOCK_SPECS);
  const specs = robot.specList;
  const idByPath = new Map<string, number>();
  specs.forEach((spec, index) => idByPath.set(spec.path, index + 1));
  const pathById = new Map([...idByPath].map(([path, id]) => [id, path]));
  const lastSent = new Map<number, SignalValue>();
  const lastLock = new Map<number, LockState>();

  let timer: ReturnType<typeof setInterval> | null = null;
  let sink: SourceSink | null = null;
  let cycle = 0;
  let pending: Promise<void> = Promise.resolve();

  const toReply = (
    op: CommandReply['op'],
    signalId: number,
    result: { accepted: boolean; reason: string | null }
  ): CommandReply => ({
    op,
    signalId,
    accepted: result.accepted,
    reason: result.reason,
    receivedAtMs: Date.now()
  });

  const commands: RobotCommands = {
    async run(signalId) {
      const path = pathById.get(signalId);
      return toReply('run', signalId, path ? robot.run(path) : { accepted: false, reason: 'rejected: unknown signal' });
    },
    async cancel(signalId) {
      const path = pathById.get(signalId);
      return toReply('cancel', signalId, path ? robot.cancel(path) : { accepted: false, reason: 'rejected: unknown signal' });
    },
    async write(signalId, value) {
      const path = pathById.get(signalId);
      return toReply('write', signalId, path ? robot.write(path, value) : { accepted: false, reason: 'rejected: unknown signal' });
    }
  };

  /** Dictionary and metadata, sent once on start. */
  function bootstrapBytes(): Uint8Array {
    const entries = specs.map((spec, index) => ({
      signalId: index + 1,
      path: spec.path,
      typeTag: spec.typeTag,
      unit: spec.unit,
      role: spec.role,
      writable: spec.writable,
      persisted: spec.persisted,
      lockGroup: spec.lockGroup
    }));
    return concat([
      encodeMetadata({ robot: 'Mock robot', event: 'Simulation', athenaVersion: 'mock', groups: MECHANISM_GROUPS.join(',') }),
      encodeDictionary(entries),
      encodeMarker(0, MARKER_KIND.BOOT, 'mock robot started')
    ]);
  }

  /** One cycle: changed values (keyframe every KEYFRAME_EVERY cycles), lock transitions and match markers. */
  function cycleBytes(events: { kind: 'match_start' | 'match_end' }[]): Uint8Array {
    const timestampMicros = Math.round(robot.time * 1_000_000);
    const keyframe = cycle % KEYFRAME_EVERY === 0;
    const entries: FrameInput['entries'] = [];
    for (const spec of specs) {
      const id = idByPath.get(spec.path) as number;
      const value = robot.value(spec.path);
      if (keyframe || lastSent.get(id) !== value) {
        entries.push({ signalId: id, typeTag: spec.typeTag, value });
        lastSent.set(id, value);
      }
    }
    const parts: Uint8Array[] = [];
    for (const event of events) {
      parts.push(
        encodeMarker(
          timestampMicros,
          event.kind === 'match_start' ? MARKER_KIND.TELEOP_START : MARKER_KIND.MATCH_END,
          event.kind === 'match_start' ? 'mock match started' : 'mock match ended'
        )
      );
    }
    parts.push(encodeFrame({ keyframe, cycleTimestampMicros: timestampMicros, sequence: cycle, entries }));
    cycle++;
    return concat(parts);
  }

  function emitLockStates(): void {
    if (!sink) return;
    for (const spec of specs) {
      if (!spec.writable) continue;
      const id = idByPath.get(spec.path) as number;
      const state = robot.lockStateOf(spec.path);
      if (lastLock.get(id) !== state) {
        lastLock.set(id, state);
        sink.lockState(id, state);
      }
    }
  }

  /** Decodes bytes in order and forwards the records. Chained so frames never overtake each other. */
  function deliver(bytes: Uint8Array, after: () => void = () => undefined): void {
    pending = pending.then(() =>
      decodeRecords(bytes).then((records) => {
        sink?.records(records);
        after();
      })
    );
  }

  function tick(): void {
    const events = robot.step(tickMs / 1000);
    deliver(cycleBytes(events), emitLockStates);
  }

  return {
    name: 'mock',
    commands,
    start(nextSink: SourceSink) {
      sink = nextSink;
      cycle = 0;
      lastSent.clear();
      lastLock.clear();
      deliver(bootstrapBytes(), emitLockStates);
      timer = setInterval(tick, tickMs);
    },
    stop() {
      if (timer !== null) clearInterval(timer);
      timer = null;
      sink = null;
    }
  };
}
