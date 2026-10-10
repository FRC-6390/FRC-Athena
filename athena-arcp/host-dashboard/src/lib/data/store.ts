import type { CommandReply } from './commands';
import { FrameAssembler } from './frames';
import { RingBuffer } from './ring';
import {
  LOCK_PATH_PREFIX,
  type DecodedRecord,
  type Frame,
  type LockState,
  type Marker,
  type Sample,
  type SignalDescriptor
} from './types';

export const DEFAULT_HISTORY_CAPACITY = 2048;
export const DEFAULT_MARKER_CAPACITY = 5000;

export type StoreOptions = {
  /** Samples kept per signal (ring buffer). */
  historyCapacity?: number;
  markerCapacity?: number;
};

export type StoreListener = () => void;

/**
 * Client value store. Holds the dictionary, the latest value and a bounded history per signal,
 * the marker list and last command replies. Listeners are notified once per applied record; UI code
 * should coalesce notifications (see `live.svelte.ts`).
 */
export class ValueStore {
  private readonly historyCapacity: number;
  private readonly markerCapacity: number;
  private readonly signals = new Map<number, SignalDescriptor>();
  private readonly byPath = new Map<string, SignalDescriptor>();
  private readonly latest = new Map<number, Sample>();
  private readonly history = new Map<number, RingBuffer<Sample>>();
  private readonly replies = new Map<number, CommandReply>();
  private readonly listeners = new Set<StoreListener>();
  private readonly assembler = new FrameAssembler();
  private markerList: Marker[] = [];
  private metadataPairs: Record<string, string> = {};
  private lastSequence: number | null = null;

  /** Incremented on every applied record. Cheap change detection for UI. */
  version = 0;
  /** Frame sequence gaps seen (frames lost in transit). */
  sequenceGaps = 0;
  /** Frame entries whose signal id has no dictionary entry yet. */
  unknownEntries = 0;

  constructor(options: StoreOptions = {}) {
    this.historyCapacity = options.historyCapacity ?? DEFAULT_HISTORY_CAPACITY;
    this.markerCapacity = options.markerCapacity ?? DEFAULT_MARKER_CAPACITY;
  }

  subscribe(listener: StoreListener): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private notify(): void {
    this.version++;
    for (const listener of this.listeners) listener();
  }

  apply(record: DecodedRecord): void {
    switch (record.kind) {
      case 'dictionary':
        this.applyDictionary(record.entries);
        break;
      case 'frame':
        this.applyFrame(record.frame);
        break;
      case 'marker':
        this.pushMarker(record.marker);
        break;
      case 'metadata':
        this.metadataPairs = { ...this.metadataPairs, ...record.pairs };
        break;
      case 'struct_schema':
        // Schemas are resolved by the decoder when a stream is read as a whole.
        break;
    }
    this.notify();
  }

  applyAll(records: Iterable<DecodedRecord>): void {
    for (const record of records) this.apply(record);
  }

  /** Replaces the dictionary. History of unchanged ids is kept. */
  private applyDictionary(entries: SignalDescriptor[]): void {
    this.signals.clear();
    this.byPath.clear();
    for (const entry of entries) {
      this.signals.set(entry.signalId, entry);
      this.byPath.set(entry.path, entry);
      if (!this.history.has(entry.signalId)) {
        this.history.set(entry.signalId, new RingBuffer<Sample>(this.historyCapacity));
      }
    }
  }

  private applyFrame(incoming: Frame): void {
    const frame = this.assembler.push(incoming);
    if (!frame) return;
    if (this.lastSequence !== null && frame.sequence > this.lastSequence + 1) {
      this.sequenceGaps += frame.sequence - this.lastSequence - 1;
    }
    this.lastSequence = frame.sequence;
    for (const entry of frame.entries) {
      if (!this.signals.has(entry.signalId)) {
        this.unknownEntries++;
        continue;
      }
      const sample: Sample = { timestampMicros: frame.cycleTimestampMicros, value: entry.value };
      this.latest.set(entry.signalId, sample);
      this.history.get(entry.signalId)?.push(sample);
    }
  }

  private pushMarker(marker: Marker): void {
    this.markerList.push(marker);
    if (this.markerList.length > this.markerCapacity) {
      this.markerList.splice(0, this.markerList.length - this.markerCapacity);
    }
  }

  /** Stores the robot's reply for the signal (latest reply wins). */
  recordReply(reply: CommandReply): void {
    this.replies.set(reply.signalId, reply);
    this.notify();
  }

  reply(signalId: number): CommandReply | undefined {
    return this.replies.get(signalId);
  }

  // Queries -----------------------------------------------------------------

  descriptor(signalId: number): SignalDescriptor | undefined {
    return this.signals.get(signalId);
  }

  descriptorByPath(path: string): SignalDescriptor | undefined {
    return this.byPath.get(path);
  }

  /** All dictionary entries, sorted by path. */
  allSignals(): SignalDescriptor[] {
    return [...this.signals.values()].sort((a, b) => a.path.localeCompare(b.path));
  }

  latestSample(signalId: number): Sample | undefined {
    return this.latest.get(signalId);
  }

  /**
   * Lock state of a writable signal (A2, E1-W "Lock state on the wire"): the role-7 boolean at
   * `athena/locks/<lockGroup>` (true = locked). `unknown` when the signal has no lock group or the lock
   * signal has not been seen. The result is informational; it never blocks a write (E5).
   */
  lockStateOf(signalId: number): LockState {
    const descriptor = this.signals.get(signalId);
    if (!descriptor || !descriptor.writable || descriptor.lockGroup === '') return 'unknown';
    const lock = this.byPath.get(LOCK_PATH_PREFIX + descriptor.lockGroup);
    if (!lock) return 'unknown';
    const sample = this.latest.get(lock.signalId);
    if (!sample || typeof sample.value !== 'boolean') return 'unknown';
    return sample.value ? 'locked' : 'open';
  }

  /** Samples oldest to newest. */
  samples(signalId: number): Sample[] {
    return this.history.get(signalId)?.toArray() ?? [];
  }

  markers(): readonly Marker[] {
    return this.markerList;
  }

  metadata(): Readonly<Record<string, string>> {
    return this.metadataPairs;
  }
}
