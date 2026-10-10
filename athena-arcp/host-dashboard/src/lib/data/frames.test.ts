import { describe, expect, test } from 'bun:test';
import { FrameAssembler } from './frames';
import { decodeDictionary, decodeFrame } from './decoder';
import { encodeDictionary, encodeFrame } from './encoder';
import { WireReader, WireWriter } from './wire-io';
import { ROLE_BY_CODE, TYPE_TAG, type Frame } from './types';

function fragment(sequence: number, index: number, count: number, signalIds: number[], keyframe = false): Frame {
  return {
    version: 2,
    keyframe,
    fragmented: true,
    cycleTimestampMicros: 500,
    sequence,
    fragmentIndex: index,
    fragmentCount: count,
    entries: signalIds.map((signalId) => ({ signalId, typeTag: TYPE_TAG.f64, value: signalId }))
  };
}

describe('FrameAssembler', () => {
  test('whole frames pass through unchanged', () => {
    const assembler = new FrameAssembler();
    const frame = fragment(1, 0, 1, [1]);
    frame.fragmented = false;
    expect(assembler.push(frame)).toBe(frame);
  });

  test('a fragment set completes only when every index arrived, in any order', () => {
    const assembler = new FrameAssembler();
    expect(assembler.push(fragment(4, 2, 3, [30]))).toBeNull();
    expect(assembler.push(fragment(4, 0, 3, [10, 11]))).toBeNull();
    const whole = assembler.push(fragment(4, 1, 3, [20], true));
    expect(whole).not.toBeNull();
    expect(whole!.sequence).toBe(4);
    expect(whole!.fragmented).toBe(false);
    expect(whole!.keyframe).toBe(true);
    expect(whole!.entries.map((e) => e.signalId)).toEqual([10, 11, 20, 30]);
    expect(assembler.openSets).toBe(0);
  });

  test('duplicate fragments do not complete a set early', () => {
    const assembler = new FrameAssembler();
    expect(assembler.push(fragment(9, 0, 2, [1]))).toBeNull();
    expect(assembler.push(fragment(9, 0, 2, [1]))).toBeNull();
    expect(assembler.push(fragment(9, 1, 2, [2]))?.entries).toHaveLength(2);
  });

  test('a newer completed frame drops older incomplete sets', () => {
    const assembler = new FrameAssembler();
    expect(assembler.push(fragment(1, 0, 2, [1]))).toBeNull();
    expect(assembler.push(fragment(2, 0, 1, [2]))?.sequence).toBe(2);
    expect(assembler.openSets).toBe(0);
    expect(assembler.droppedFrames).toBe(1);
  });

  test('the number of open sets is bounded', () => {
    const assembler = new FrameAssembler(2);
    for (let seq = 0; seq < 5; seq++) assembler.push(fragment(seq, 0, 2, [seq]));
    expect(assembler.openSets).toBe(2);
    expect(assembler.droppedFrames).toBe(3);
  });
});

describe('dictionary role codes (E1-W, normative)', () => {
  test('codes 0-8 map to the named roles', () => {
    expect([...ROLE_BY_CODE]).toEqual([
      'unspecified',
      'state',
      'value',
      'control',
      'device',
      'action',
      'hook',
      'lock',
      'system'
    ]);
  });

  test('round-trips every role and maps unknown codes to unspecified', () => {
    const entries = ROLE_BY_CODE.map((role, i) => ({
      signalId: i + 1,
      path: `robot/x/Values/r${i}`,
      typeTag: TYPE_TAG.bool,
      role
    }));
    const decoded = decodeDictionary(encodeDictionary(entries).subarray(3));
    expect(decoded.map((e) => e.role)).toEqual([...ROLE_BY_CODE]);

    // Code 42 is not defined yet: decodes as unspecified.
    const w = new WireWriter()
      .varint(1)
      .varint(1)
      .string('robot/x/Values/z')
      .u8(TYPE_TAG.bool)
      .string('')
      .u8(42)
      .u8(0)
      .string('')
      .varint(0);
    expect(decodeDictionary(w.toBytes())[0].role).toBe('unspecified');
  });
});

describe('i64 and frame decoding', () => {
  test('zigzag bigint round-trips across the safe range boundary', () => {
    for (const value of [0n, -1n, 1n, 9007199254740991n, -9007199254740992n, 9007199254740993n, -(2n ** 63n)]) {
      const bytes = new WireWriter().zigzagBig(value).toBytes();
      expect(new WireReader(bytes).zigzagBig()).toBe(value);
    }
  });

  test('a fragmented frame header is decoded with its fragment fields', () => {
    const bytes = encodeFrame({
      cycleTimestampMicros: 1234,
      sequence: 3,
      fragment: { index: 1, count: 4 },
      entries: [{ signalId: 5, typeTag: TYPE_TAG.f64, value: 2.5 }]
    });
    // Strip the 6-byte record envelope (type + 1-byte length) to get the frame payload.
    const frame = decodeFrame(bytes.subarray(2));
    expect(frame.fragmented).toBe(true);
    expect(frame.fragmentIndex).toBe(1);
    expect(frame.fragmentCount).toBe(4);
    expect(frame.sequence).toBe(3);
  });
});
