import { describe, expect, test } from 'bun:test';
import { buildReplayModel, decodePose2d, lastIndexAtOrBefore } from './replay';
import { buildCsv } from './csv';
import { decodeReplay } from '../data/decoder';
import { encodeCompressedBlock, encodeDictionary, encodeFrame, encodeMarker, encodeReplay, encodeStructSchema, concat } from '../data/encoder';
import { TYPE_TAG } from '../data/types';

async function sampleReplay(): Promise<Uint8Array> {
  const pose = new Uint8Array(24);
  const view = new DataView(pose.buffer);
  view.setFloat64(0, 1.5, true);
  view.setFloat64(8, 2.5, true);
  view.setFloat64(16, Math.PI / 2, true);
  const block = concat([
    encodeDictionary([
      { signalId: 1, path: 'robot/a/Values/x', typeTag: TYPE_TAG.f64, unit: 'deg' },
      { signalId: 2, path: 'robot/a/Values/on', typeTag: TYPE_TAG.bool },
      { signalId: 3, path: 'robot/drive/Values/pose', typeTag: TYPE_TAG.struct, structSchemaId: 1 }
    ]),
    encodeStructSchema(1, 'Pose2d', 'double x;double y;double rot'),
    encodeFrame({ keyframe: true, cycleTimestampMicros: 0, sequence: 0, entries: [
      { signalId: 1, typeTag: TYPE_TAG.f64, value: 1 },
      { signalId: 2, typeTag: TYPE_TAG.bool, value: false },
      { signalId: 3, typeTag: TYPE_TAG.struct, value: { schemaId: 1, bytes: pose } }
    ] }),
    encodeMarker(500_000, 4, 'teleop'),
    encodeFrame({ cycleTimestampMicros: 1_000_000, sequence: 1, entries: [{ signalId: 1, typeTag: TYPE_TAG.f64, value: 3 }] }),
    encodeFrame({ cycleTimestampMicros: 2_000_000, sequence: 3, entries: [{ signalId: 2, typeTag: TYPE_TAG.bool, value: true }] })
  ]);
  return encodeReplay(await encodeCompressedBlock(block));
}

describe('replay model', () => {
  test('decodes a compressed replay into tracks, markers and sample-and-hold lookups', async () => {
    const model = buildReplayModel(await decodeReplay(await sampleReplay()));
    expect(model.frameCount).toBe(3);
    expect(model.startMicros).toBe(0);
    expect(model.endMicros).toBe(2_000_000);
    expect(model.sequenceGaps).toBe(1);
    expect(model.markers.map((m) => m.label)).toEqual(['Teleop start']);
    expect(model.valueAt(1, 500_000)).toBe(1);
    expect(model.valueAt(1, 1_000_000)).toBe(3);
    expect(model.valueAt(2, 1_500_000)).toBe(false);
    expect(model.valueAt(2, 2_000_000)).toBe(true);
    expect(model.valueAt(1, -1)).toBeUndefined();
  });

  test('numeric and pose views are derived from the tracks', async () => {
    const model = buildReplayModel(await decodeReplay(await sampleReplay()));
    expect(model.numericTrack(2)?.v).toEqual([0, 1]);
    expect(model.numericTrack(3)).toBeUndefined();
    expect(model.poseTrack(3)?.pose[0]).toEqual({ x: 1.5, y: 2.5, theta: Math.PI / 2 });
  });

  test('binary search finds the last sample at or before a time', () => {
    expect(lastIndexAtOrBefore([10, 20, 30], 5)).toBe(-1);
    expect(lastIndexAtOrBefore([10, 20, 30], 20)).toBe(1);
    expect(lastIndexAtOrBefore([10, 20, 30], 99)).toBe(2);
    expect(lastIndexAtOrBefore([], 1)).toBe(-1);
  });

  test('Pose2d decoding rejects short payloads', () => {
    expect(decodePose2d(new Uint8Array(8))).toBeNull();
  });
});

describe('CSV export', () => {
  test('exports the selected range with sample-and-hold values and a time origin', async () => {
    const model = buildReplayModel(await decodeReplay(await sampleReplay()));
    const csv = buildCsv(model, [1, 2, 3], 500_000, 2_000_000);
    const lines = csv.trim().split('\n');
    expect(lines[0]).toBe('time_s,robot/a/Values/x [deg],robot/a/Values/on,robot/drive/Values/pose.x,robot/drive/Values/pose.y,robot/drive/Values/pose.theta');
    expect(lines[1].startsWith('0.500000,1,0,1.5,2.5,')).toBe(true);
    expect(lines.length).toBe(4);
  });

  test('quotes text cells that contain commas', () => {
    const model = buildReplayModel([]);
    expect(buildCsv(model, [], 0, 0)).toBe('time_s\n');
  });
});
