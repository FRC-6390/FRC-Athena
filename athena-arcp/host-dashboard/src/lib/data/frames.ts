import type { Frame, FrameEntry } from './types';

/** Fragment sets kept open at once. Older incomplete sets are dropped (their frame is lost). */
export const MAX_PENDING_FRAMES = 8;

type Pending = {
  sequence: number;
  timestampMicros: number;
  keyframe: boolean;
  count: number;
  parts: (FrameEntry[] | undefined)[];
  received: number;
};

/**
 * Reassembles fragmented FRAME records (E1-W Frame: `fragmentIndex`, `fragmentCount`, fragments share a
 * `sequence`). Whole frames pass straight through. A set completes when all of its fragments arrived; the
 * result is a single unfragmented frame whose entries are the fragments concatenated in index order.
 */
export class FrameAssembler {
  private readonly pending = new Map<number, Pending>();
  /** Fragment sets dropped without completing (eviction or superseded by a newer frame). */
  droppedFrames = 0;

  constructor(private readonly maxPending = MAX_PENDING_FRAMES) {}

  /** Returns the complete frame, or null while fragments are still missing. */
  push(frame: Frame): Frame | null {
    if (!frame.fragmented || frame.fragmentCount <= 1) {
      this.supersede(frame.sequence);
      return frame;
    }
    if (frame.fragmentIndex >= frame.fragmentCount) {
      // Malformed fragment: ignore it rather than corrupt a set.
      return null;
    }
    let set = this.pending.get(frame.sequence);
    if (!set || set.count !== frame.fragmentCount) {
      if (set) this.pending.delete(frame.sequence);
      set = {
        sequence: frame.sequence,
        timestampMicros: frame.cycleTimestampMicros,
        keyframe: frame.keyframe,
        count: frame.fragmentCount,
        parts: new Array(frame.fragmentCount),
        received: 0
      };
      this.pending.set(frame.sequence, set);
      this.evictOldest();
    }
    if (set.parts[frame.fragmentIndex] === undefined) {
      set.parts[frame.fragmentIndex] = frame.entries;
      set.received++;
    }
    set.keyframe = set.keyframe || frame.keyframe;
    if (set.received < set.count) return null;

    this.pending.delete(frame.sequence);
    this.supersede(frame.sequence);
    const entries: FrameEntry[] = [];
    for (const part of set.parts) if (part) entries.push(...part);
    return {
      version: frame.version,
      keyframe: set.keyframe,
      fragmented: false,
      cycleTimestampMicros: set.timestampMicros,
      sequence: set.sequence,
      fragmentIndex: 0,
      fragmentCount: 1,
      entries
    };
  }

  /** Drops open sets older than a frame that has now completed; they cannot finish in order. */
  private supersede(sequence: number): void {
    for (const [seq] of this.pending) {
      if (seq < sequence) {
        this.pending.delete(seq);
        this.droppedFrames++;
      }
    }
  }

  private evictOldest(): void {
    while (this.pending.size > this.maxPending) {
      const oldest = this.pending.keys().next().value as number;
      this.pending.delete(oldest);
      this.droppedFrames++;
    }
  }

  get openSets(): number {
    return this.pending.size;
  }
}
