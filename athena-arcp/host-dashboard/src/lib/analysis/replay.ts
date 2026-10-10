/**
 * In-memory model of a `.athrp` replay for the Analysis workspace. Built from decoded records: frames
 * are reassembled, every signal gets a time-ordered track of changed values, and lookups are
 * sample-and-hold (the value holds until the next change). Pure TypeScript, no DOM.
 */
import type { DecodedRecord, Marker, SignalDescriptor, SignalValue } from '../data/types';
import { FrameAssembler } from '../data/frames';

export type Track = {
  /** Sample times in robot micros, ascending. */
  t: number[];
  v: SignalValue[];
};

export type Pose2dValue = { x: number; y: number; theta: number };

export type ReplayModel = {
  metadata: Record<string, string>;
  /** Latest dictionary entry per signal id. */
  descriptors: Map<number, SignalDescriptor>;
  tracks: Map<number, Track>;
  markers: Marker[];
  startMicros: number;
  endMicros: number;
  frameCount: number;
  sequenceGaps: number;
  /** Frame entries whose signal id has no dictionary entry. */
  unknownEntries: number;
  descriptor(signalId: number): SignalDescriptor | undefined;
  /** Dictionary entries sorted by path. */
  signals(): SignalDescriptor[];
  /** Value at `micros` (sample-and-hold), or undefined before the first sample. */
  valueAt(signalId: number, micros: number): SignalValue | undefined;
  /** Numeric view of a track: bools as 0/1, i64 as number. Undefined for non-numeric signals. */
  numericTrack(signalId: number): { t: number[]; v: number[] } | undefined;
  /** Pose2d view of a struct:Pose2d track. */
  poseTrack(signalId: number): { t: number[]; pose: Pose2dValue[] } | undefined;
};

/** WPILib Pose2d struct layout: x, y, rotation (radians), little-endian doubles. */
export function decodePose2d(bytes: Uint8Array): Pose2dValue | null {
  if (bytes.length < 24) return null;
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return {
    x: view.getFloat64(0, true),
    y: view.getFloat64(8, true),
    theta: view.getFloat64(16, true)
  };
}

/** Index of the last sample at or before `micros`, or -1. */
export function lastIndexAtOrBefore(times: readonly number[], micros: number): number {
  let lo = 0;
  let hi = times.length - 1;
  let found = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >>> 1;
    if (times[mid] <= micros) {
      found = mid;
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  return found;
}

export function buildReplayModel(records: readonly DecodedRecord[]): ReplayModel {
  const descriptors = new Map<number, SignalDescriptor>();
  const tracks = new Map<number, Track>();
  const markers: Marker[] = [];
  const metadata: Record<string, string> = {};
  const assembler = new FrameAssembler();
  let frameCount = 0;
  let sequenceGaps = 0;
  let unknownEntries = 0;
  let lastSequence: number | null = null;
  let startMicros = Number.POSITIVE_INFINITY;
  let endMicros = Number.NEGATIVE_INFINITY;

  for (const record of records) {
    switch (record.kind) {
      case 'dictionary':
        descriptors.clear();
        for (const entry of record.entries) descriptors.set(entry.signalId, entry);
        break;
      case 'metadata':
        Object.assign(metadata, record.pairs);
        break;
      case 'struct_schema':
        // A schema may arrive after the dictionary that uses it; re-name those struct signals.
        for (const [id, d] of descriptors) {
          if (d.structSchemaId === record.schema.schemaId && d.type.startsWith('struct:#')) {
            descriptors.set(id, { ...d, type: `struct:${record.schema.typeName}` });
          }
        }
        break;
      case 'marker':
        markers.push(record.marker);
        break;
      case 'frame': {
        const frame = assembler.push(record.frame);
        if (!frame) break;
        if (lastSequence !== null && frame.sequence > lastSequence + 1) {
          sequenceGaps += frame.sequence - lastSequence - 1;
        }
        lastSequence = frame.sequence;
        frameCount++;
        const t = frame.cycleTimestampMicros;
        startMicros = Math.min(startMicros, t);
        endMicros = Math.max(endMicros, t);
        for (const entry of frame.entries) {
          if (!descriptors.has(entry.signalId)) {
            unknownEntries++;
            continue;
          }
          let track = tracks.get(entry.signalId);
          if (!track) {
            track = { t: [], v: [] };
            tracks.set(entry.signalId, track);
          }
          track.t.push(t);
          track.v.push(entry.value);
        }
        break;
      }
      default:
        break;
    }
  }
  markers.sort((a, b) => a.timestampMicros - b.timestampMicros);
  if (frameCount === 0) {
    startMicros = 0;
    endMicros = 0;
  }

  return {
    metadata,
    descriptors,
    tracks,
    markers,
    startMicros,
    endMicros,
    frameCount,
    sequenceGaps,
    unknownEntries,
    descriptor: (id) => descriptors.get(id),
    signals: () => [...descriptors.values()].sort((a, b) => a.path.localeCompare(b.path)),
    valueAt(id, micros) {
      const track = tracks.get(id);
      if (!track) return undefined;
      const i = lastIndexAtOrBefore(track.t, micros);
      return i < 0 ? undefined : track.v[i];
    },
    numericTrack(id) {
      const track = tracks.get(id);
      const descriptor = descriptors.get(id);
      if (!track || !descriptor) return undefined;
      if (descriptor.type !== 'bool' && descriptor.type !== 'i64' && descriptor.type !== 'f64') return undefined;
      const v = track.v.map((x) => (typeof x === 'boolean' ? (x ? 1 : 0) : Number(x)));
      return { t: track.t, v };
    },
    poseTrack(id) {
      const track = tracks.get(id);
      const descriptor = descriptors.get(id);
      if (!track || !descriptor || descriptor.type !== 'struct:Pose2d') return undefined;
      const pose: Pose2dValue[] = [];
      const t: number[] = [];
      track.v.forEach((value, i) => {
        if (typeof value === 'object' && value !== null && 'bytes' in value) {
          const decoded = decodePose2d(value.bytes);
          if (decoded) {
            pose.push(decoded);
            t.push(track.t[i]);
          }
        }
      });
      return { t, pose };
    }
  };
}
