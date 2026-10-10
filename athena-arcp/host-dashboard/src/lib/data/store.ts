import type { CommandReply } from './commands';
import { RingBuffer } from './ring';
import {
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
 * should coalesce notifications (see `svelte.svelte.ts`).
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

  /** Replaces the dictionary. Lock state of unchanged ids is kept. */
  private applyDictionary(entries: SignalDescriptor[]): void {
    const previous = new Map(this.signals);
    this.signals.clear();
    this.byPath.clear();
    for (const entry of entries) {
      const prior = previous.get(entry.signalId);
      const descriptor = prior && prior.path === entry.path ? { ...entry, lockState: prior.lockState } : entry;
      this.signals.set(descriptor.signalId, descriptor);
      this.byPath.set(descriptor.path, descriptor);
      if (!this.history.has(descriptor.signalId)) {
        this.history.set(descriptor.signalId, new RingBuffer<Sample>(this.historyCapacity));
      }
    }
  }

  private applyFrame(frame: Frame): void {
    if (this.lastSequence !== null && frame.sequence > this.lastSequence + 1 && !frame.fragmented) {
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

  /** Sets the lock state of a writable signal (A2). Sources call this; UI only displays it. */
  setLockState(signalId: number, state: LockState): void {
    const descriptor = this.signals.get(signalId);
    if (!descriptor || descriptor.lockState === state) return;
    const updated = { ...descriptor, lockState: state };
    this.signals.set(signalId, updated);
    this.byPath.set(updated.path, updated);
    this.notify();
  }

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
