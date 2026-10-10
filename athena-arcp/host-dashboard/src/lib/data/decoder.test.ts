import { describe, expect, test } from 'bun:test';
import {
  createDecodeContext,
  decodeFrame,
  decodeRecords,
  decodeReplay,
  markerKindLabel
} from './decoder';
import {
  concat,
  encodeCompressedBlock,
  encodeDictionary,
  encodeFrame,
  encodeMarker,
  encodeMetadata,
  encodeRecord,
  encodeReplay,
  encodeStructSchema
} from './encoder';
import { WireError, WireReader, WireWriter } from './wire-io';
import { TYPE_TAG } from './types';

const bytesOf = (...values: number[]) => Uint8Array.from(values);

describe('wire primitives', () => {
  test('varint and zigzag round-trip, including values above 2^32', () => {
    const values = [0, 1, 127, 128, 300, 16_384, 2 ** 32 + 5, 2 ** 52];
    const w = new WireWriter();
    for (const v of values) w.varint(v);
    for (const v of [0, -1, 1, -64, 63, -(2 ** 40)]) w.zigzag(v);
    const r = new WireReader(w.toBytes());
    for (const v of values) expect(r.varint()).toBe(v);
    for (const v of [0, -1, 1, -64, 63, -(2 ** 40)]) expect(r.zigzag()).toBe(v);
    expect(r.remaining).toBe(0);
  });

  test('varint encodes 300 as the spec LEB128 bytes', () => {
    expect(new WireWriter().varint(300).toBytes()).toEqual(bytesOf(0xac, 0x02));
  });

  test('u64 and f64 are little-endian', () => {
    const w = new WireWriter().u64(1_000_000).f64(1.5);
    expect(w.toBytes()).toEqual(
      bytesOf(0x40, 0x42, 0x0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf8, 0x3f)
    );
    const r = new WireReader(w.toBytes());
    expect(r.u64()).toBe(1_000_000);
    expect(r.f64()).toBe(1.5);
  });

  test('truncated input raises WireError instead of reading past the end', () => {
    const r = new WireReader(bytesOf(0x05, 0x41));
    expect(() => r.string()).toThrow(WireError);
  });

  test('element counts larger than the remaining data are rejected', () => {
    const r = new WireReader(bytesOf(0x7f));
    expect(() => r.count()).toThrow(WireError);
  });
});

describe('hand-built E1-W vectors', () => {
  test('DICTIONARY record with one f64 entry', async () => {
    // "robot/x/Values/v" is 16 bytes. Payload: count 1, id 1, path, tag 3 (f64), unit "",
    // role 2 (value), flags 3 (writable|persisted), lockGroup "", structSchemaId 0.
    const path = [0x72, 0x6f, 0x62, 0x6f, 0x74, 0x2f, 0x78, 0x2f, 0x56, 0x61, 0x6c, 0x75, 0x65, 0x73, 0x2f, 0x76];
    const payload = [0x01, 0x01, 0x10, ...path, 0x03, 0x00, 0x02, 0x03, 0x00, 0x00];
    expect(payload.length).toBe(0x19);
    const records = await decodeRecords(bytesOf(0x01, 0x19, ...payload));
    expect(records).toHaveLength(1);
    const [dict] = records;
    expect(dict.kind).toBe('dictionary');
    if (dict.kind !== 'dictionary') throw new Error('unreachable');
    expect(dict.entries).toEqual([
      {
        signalId: 1,
        path: 'robot/x/Values/v',
        mechanismPath: 'robot/x',
        localPath: 'Values/v',
        name: 'v',
        type: 'f64',
        typeTag: 3,
        unit: '',
        role: 'value',
        writable: true,
        persisted: true,
        lockGroup: '',
        lockState: 'unknown',
        structSchemaId: 0
      }
    ]);
  });

  test('FRAME record: keyframe, two entries (f64 and bool)', () => {
    const payload = bytesOf(
      0x02, // version
      0x01, // flags: keyframe
      0x40, 0x42, 0x0f, 0, 0, 0, 0, 0, // cycle timestamp 1_000_000 us
      0x05, 0, 0, 0, // sequence 5
      0, 0, // fragment index
      1, 0, // fragment count
      0x02, // entry count
      0x01, 0x03, 0, 0, 0, 0, 0, 0, 0xf8, 0x3f, // id 1, f64 1.5
      0x02, 0x01, 0x01 // id 2, bool true
    );
    const frame = decodeFrame(payload);
    expect(frame.keyframe).toBe(true);
    expect(frame.fragmented).toBe(false);
    expect(frame.cycleTimestampMicros).toBe(1_000_000);
    expect(frame.sequence).toBe(5);
    expect(frame.entries).toEqual([
      { signalId: 1, typeTag: 3, value: 1.5 },
      { signalId: 2, typeTag: 1, value: true }
    ]);
  });

  test('MARKER record: negative timestamp, FAULT kind, message, no signal', async () => {
    // zigzag(-1) = 1, kind 7, message "hi", signalId 0 (none).
    const records = await decodeRecords(bytesOf(0x03, 0x06, 0x01, 0x07, 0x02, 0x68, 0x69, 0x00));
    expect(records).toEqual([
      {
        kind: 'marker',
        marker: {
          timestampMicros: -1,
          kind: 7,
          label: 'Fault',
          message: 'hi',
          signalId: null
        }
      }
    ]);
  });

  test('METADATA record', async () => {
    const records = await decodeRecords(bytesOf(0x04, 0x05, 0x01, 0x01, 0x6b, 0x01, 0x76));
    expect(records).toEqual([{ kind: 'metadata', pairs: { k: 'v' } }]);
  });

  test('replay file header is checked and the version is enforced', async () => {
    const good = encodeReplay(encodeMetadata({ event: 'practice' }));
    expect(await decodeReplay(good)).toEqual([{ kind: 'metadata', pairs: { event: 'practice' } }]);

    const badMagic = good.slice();
    badMagic[0] = 0x42;
    await expect(decodeReplay(badMagic)).rejects.toThrow(/magic/);

    const badVersion = good.slice();
    badVersion[6] = 2;
    await expect(decodeReplay(badVersion)).rejects.toThrow(/version/);
  });

  test('unknown record types are skipped by their declared length', async () => {
    const unknown = bytesOf(0x7e, 0x02, 0xaa, 0xbb);
    const metadata = encodeMetadata({ a: 'b' });
    const records = await decodeRecords(concat([unknown, metadata]));
    expect(records).toEqual([{ kind: 'metadata', pairs: { a: 'b' } }]);
  });
});

describe('frames', () => {
  test('every typed value survives encode and decode', () => {
    const schemaBytes = bytesOf(1, 2, 3, 4);
    const input = {
      keyframe: true,
      cycleTimestampMicros: 123_456_789,
      sequence: 99,
      entries: [
        { signalId: 1, typeTag: TYPE_TAG.bool, value: false },
        { signalId: 2, typeTag: TYPE_TAG.i64, value: -42 },
        { signalId: 3, typeTag: TYPE_TAG.f64, value: -0.25 },
        { signalId: 4, typeTag: TYPE_TAG.string, value: 'héllo' },
        { signalId: 5, typeTag: TYPE_TAG['bool[]'], value: [true, false, true] },
        { signalId: 6, typeTag: TYPE_TAG['i64[]'], value: [-1, 0, 7] },
        { signalId: 7, typeTag: TYPE_TAG['f64[]'], value: [1.25, -3.5] },
        { signalId: 8, typeTag: TYPE_TAG['string[]'], value: ['a', '', 'ccc'] },
        { signalId: 9, typeTag: TYPE_TAG.struct, value: { schemaId: 4, bytes: schemaBytes } }
      ]
    };
    const decoded = decodeFrame(extractPayload(encodeFrame(input)));
    expect(decoded.entries).toEqual(input.entries);
    expect(decoded.cycleTimestampMicros).toBe(123_456_789);
    expect(decoded.sequence).toBe(99);
  });

  test('fragment fields are preserved', () => {
    const frame = decodeFrame(
      extractPayload(
        encodeFrame({
          cycleTimestampMicros: 1,
          sequence: 1,
          fragment: { index: 1, count: 3 },
          entries: []
        })
      )
    );
    expect(frame.fragmented).toBe(true);
    expect(frame.fragmentIndex).toBe(1);
    expect(frame.fragmentCount).toBe(3);
  });

  test('struct values are named by the schema record that precedes them', async () => {
    const schema = encodeStructSchema(4, 'Pose2d', 'double x;double y;double rot');
    const dict = encodeDictionary([
      { signalId: 1, path: 'robot/drive/State/pose', typeTag: TYPE_TAG.struct, structSchemaId: 4 }
    ]);
    const frame = encodeFrame({
      cycleTimestampMicros: 10,
      sequence: 0,
      entries: [{ signalId: 1, typeTag: TYPE_TAG.struct, value: { schemaId: 4, bytes: bytesOf(0, 0, 0, 0) } }]
    });
    const records = await decodeRecords(concat([schema, dict, frame]));
    const dictRecord = records[1];
    expect(dictRecord.kind === 'dictionary' && dictRecord.entries[0].type).toBe('struct:Pose2d');
  });
});

describe('compressed blocks', () => {
  test('a COMPRESSED_BLOCK expands to the same records as its plain contents', async () => {
    const inner = concat([
      encodeDictionary([{ signalId: 1, path: 'robot/a/State/s', typeTag: TYPE_TAG.string }]),
      encodeFrame({
        keyframe: true,
        cycleTimestampMicros: 500,
        sequence: 1,
        entries: [{ signalId: 1, typeTag: TYPE_TAG.string, value: 'idle' }]
      }),
      encodeMarker(500, 3, 'auto')
    ]);
    const plain = await decodeRecords(inner);
    const packed = await decodeRecords(await encodeCompressedBlock(inner));
    expect(packed).toEqual(plain);
    expect(packed.map((r) => r.kind)).toEqual(['dictionary', 'frame', 'marker']);
  });

  test('a block whose declared length is wrong is rejected', async () => {
    const inner = encodeMetadata({ a: 'b' });
    const block = await encodeCompressedBlock(inner);
    // Corrupt the uncompressed length varint (first payload byte, after the 0x10 record header and length).
    const corrupt = block.slice();
    corrupt[2] = corrupt[2] + 1;
    await expect(decodeRecords(corrupt)).rejects.toThrow();
  });
});

describe('misc', () => {
  test('marker kind labels cover the spec kinds and user range', () => {
    expect(markerKindLabel(2)).toBe('Match identified');
    expect(markerKindLabel(12)).toBe('Record');
    expect(markerKindLabel(150)).toBe('User 150');
    expect(markerKindLabel(42)).toBe('Unknown 42');
  });

  test('createDecodeContext starts empty', () => {
    expect(createDecodeContext().structSchemas.size).toBe(0);
  });

  test('encodeRecord wraps a payload with its type and length', () => {
    expect(encodeRecord(0x04, bytesOf(1, 2))).toEqual(bytesOf(0x04, 0x02, 1, 2));
  });
});

/** Strips the record envelope from a single encoded record, returning its payload. */
function extractPayload(record: Uint8Array): Uint8Array {
  const r = new WireReader(record);
  r.u8();
  const length = r.varint();
  return r.bytesOf(length);
}
