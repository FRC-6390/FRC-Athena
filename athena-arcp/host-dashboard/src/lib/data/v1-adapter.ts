/**
 * Adapter from the ARCP v1 Tauri client (`lib/arcp.ts`) to the data layer. Lets the new UI run against
 * a v1 robot without the v1 snapshot types leaking into components.
 *
 * v1 has no units, lock groups, timestamps per value or struct types; those fields get neutral defaults.
 */
import { deltaArcp, fireAction, setSignal, type DashboardDelta, type SignalRow } from '../arcp';
import { isWritableSignal, signalRole } from '../dashboard';
import type { CommandReply, RobotCommands } from './commands';
import type { DataSource, SourceSink } from './source';
import { describePath } from './path';
import { FRAME_VERSION } from './decoder';
import { TYPE_TAG, type DecodedRecord, type SignalDescriptor, type SignalType, type SignalValue, type TypeTag } from './types';

const KNOWN_TYPES: Record<string, TypeTag> = {
  bool: TYPE_TAG.bool,
  i64: TYPE_TAG.i64,
  f64: TYPE_TAG.f64,
  string: TYPE_TAG.string,
  'bool[]': TYPE_TAG['bool[]'],
  'i64[]': TYPE_TAG['i64[]'],
  'f64[]': TYPE_TAG['f64[]'],
  'string[]': TYPE_TAG['string[]']
};

function typeOf(raw: string): { type: SignalType; typeTag: TypeTag } {
  const typeTag = KNOWN_TYPES[raw] ?? TYPE_TAG.string;
  const type = (Object.keys(KNOWN_TYPES).find((k) => KNOWN_TYPES[k] === typeTag) ?? 'string') as SignalType;
  return { type, typeTag };
}

/** Parses a v1 display value string into the typed value for the given type. */
export function parseV1Value(typeTag: TypeTag, raw: string): SignalValue {
  switch (typeTag) {
    case TYPE_TAG.bool:
      return raw === 'true' || raw === '1';
    case TYPE_TAG.i64:
      return Number.parseInt(raw, 10) || 0;
    case TYPE_TAG.f64:
      return Number.parseFloat(raw) || 0;
    case TYPE_TAG['bool[]']:
    case TYPE_TAG['i64[]']:
    case TYPE_TAG['f64[]']:
    case TYPE_TAG['string[]']: {
      try {
        const parsed: unknown = JSON.parse(raw);
        if (Array.isArray(parsed)) return parsed as string[];
      } catch {
        /* fall through to a single-element array */
      }
      return raw.length > 0 ? [raw] : [];
    }
    default:
      return raw;
  }
}

function roleOf(row: SignalRow): SignalDescriptor['role'] {
  switch (signalRole(row)) {
    case 'action':
      return 'action';
    case 'state':
      return 'state';
    case 'tunable':
      return 'control';
    default:
      return 'value';
  }
}

export function describeV1Signal(row: SignalRow): SignalDescriptor {
  const { type, typeTag } = typeOf(row.signal_type);
  return {
    signalId: row.signal_id,
    ...describePath(row.path),
    type,
    typeTag,
    unit: '',
    role: roleOf(row),
    writable: isWritableSignal(row),
    persisted: row.durability !== 'volatile',
    lockGroup: '',
    structSchemaId: 0
  };
}

/**
 * Turns the v1 signal table into dictionary + frame records. The dictionary is re-emitted only when the
 * set of signals changes, so the store keeps history across ordinary updates.
 */
export class V1DataAdapter {
  private signature = '';
  private readonly rows = new Map<number, SignalRow>();

  /** Merges a delta (or full snapshot) and returns the records to feed into the store. */
  ingest(delta: DashboardDelta, timestampMicros: number, sequence: number): DecodedRecord[] {
    if (delta.full_snapshot) this.rows.clear();
    for (const row of delta.signals) this.rows.set(row.signal_id, row);

    const records: DecodedRecord[] = [];
    const ordered = [...this.rows.values()].sort((a, b) => a.signal_id - b.signal_id);
    const signature = ordered.map((r) => `${r.signal_id}:${r.path}:${r.signal_type}`).join('|');
    if (signature !== this.signature) {
      this.signature = signature;
      records.push({ kind: 'dictionary', entries: ordered.map(describeV1Signal) });
    }
    const changed = new Set(delta.signals.map((r) => r.signal_id));
    const entries = ordered
      .filter((row) => delta.full_snapshot || changed.has(row.signal_id))
      .map((row) => {
        const { typeTag } = typeOf(row.signal_type);
        return { signalId: row.signal_id, typeTag, value: parseV1Value(typeTag, row.value) };
      });
    records.push({
      kind: 'frame',
      frame: {
        version: FRAME_VERSION,
        keyframe: delta.full_snapshot,
        fragmented: false,
        cycleTimestampMicros: timestampMicros,
        sequence,
        fragmentIndex: 0,
        fragmentCount: 1,
        entries
      }
    });
    return records;
  }
}

/** Commands over the v1 control channel. v1 cannot cancel actions. */
export function createV1Commands(): RobotCommands {
  const reply = (
    requestId: number,
    op: CommandReply['op'],
    signalId: number,
    accepted: boolean,
    reason: string,
    actualValue: CommandReply['actualValue'] = null
  ): CommandReply => ({ requestId, op, signalId, accepted, reason, actualValue, receivedAtMs: Date.now() });
  const fail = (requestId: number, op: CommandReply['op'], signalId: number, error: unknown) =>
    reply(requestId, op, signalId, false, `rejected: ${error instanceof Error ? error.message : String(error)}`);
  return {
    run: (requestId, signalId) =>
      fireAction(signalId).then(
        () => reply(requestId, 'run', signalId, true, ''),
        (error: unknown) => fail(requestId, 'run', signalId, error)
      ),
    cancel: async (requestId, signalId) =>
      reply(requestId, 'cancel', signalId, false, 'rejected: ARCP v1 has no cancel'),
    write: (requestId, signalId, value) =>
      setSignal(signalId, typeof value === 'string' ? value : JSON.stringify(value)).then(
        () => reply(requestId, 'write', signalId, true, ''),
        (error: unknown) => fail(requestId, 'write', signalId, error)
      )
  };
}

/** Polls `deltaArcp` and feeds the store. Requires an established v1 connection (`connectArcp`). */
export function createArcpV1Source(intervalMs = 100): DataSource {
  const adapter = new V1DataAdapter();
  let timer: ReturnType<typeof setInterval> | null = null;
  let sinceUpdate: number | null = null;
  let manifest: number | null = null;
  let busy = false;
  let sink: SourceSink | null = null;

  const poll = async () => {
    if (busy || !sink) return;
    busy = true;
    try {
      const delta = await deltaArcp(sinceUpdate, manifest);
      sinceUpdate = delta.update_count;
      manifest = delta.manifest_revision;
      const records = adapter.ingest(delta, Math.round(performance.now() * 1000), delta.update_count);
      sink.records(records);
    } catch {
      // Connection errors surface through the classic status bar; keep polling quietly.
    } finally {
      busy = false;
    }
  };

  return {
    name: 'arcp-v1',
    commands: createV1Commands(),
    start(nextSink) {
      sink = nextSink;
      timer = setInterval(() => void poll(), intervalMs);
    },
    stop() {
      if (timer !== null) clearInterval(timer);
      timer = null;
      sink = null;
    }
  };
}
