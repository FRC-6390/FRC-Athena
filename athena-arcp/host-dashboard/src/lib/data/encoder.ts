/**
 * Encoder for the E1-W format. Mirrors the Java writer closely enough to build fixtures for tests and
 * to write replay snippets from the client. Not used on the hot path.
 */
import {
  DICTIONARY_FLAG_PERSISTED,
  DICTIONARY_FLAG_WRITABLE,
  FRAME_FLAG_FRAGMENT,
  FRAME_FLAG_KEYFRAME,
  FRAME_VERSION,
  REPLAY_FORMAT_VERSION,
  REPLAY_MAGIC,
  RECORD_TYPE
} from './decoder';
import { WireWriter } from './wire-io';
import {
  ROLE_BY_CODE,
  type SignalRole,
  type SignalValue,
  type TypeTag,
  TYPE_TAG
} from './types';

export type DictionaryInput = {
  signalId: number;
  path: string;
  typeTag: TypeTag;
  unit?: string;
  role?: SignalRole;
  writable?: boolean;
  persisted?: boolean;
  lockGroup?: string;
  structSchemaId?: number;
};

export type FrameInput = {
  keyframe?: boolean;
  cycleTimestampMicros: number;
  sequence: number;
  fragment?: { index: number; count: number };
  entries: { signalId: number; typeTag: TypeTag; value: SignalValue }[];
};

export function encodeRecord(recordType: number, payload: Uint8Array): Uint8Array {
  return new WireWriter().record(recordType, payload).toBytes();
}

export function encodeDictionary(entries: DictionaryInput[]): Uint8Array {
  const w = new WireWriter().varint(entries.length);
  for (const e of entries) {
    const flags =
      (e.writable ? DICTIONARY_FLAG_WRITABLE : 0) | (e.persisted ? DICTIONARY_FLAG_PERSISTED : 0);
    w.varint(e.signalId)
      .string(e.path)
      .u8(e.typeTag)
      .string(e.unit ?? '')
      .u8(ROLE_BY_CODE.indexOf(e.role ?? 'unspecified'))
      .u8(flags)
      .string(e.lockGroup ?? '')
      .varint(e.structSchemaId ?? 0);
  }
  return encodeRecord(RECORD_TYPE.DICTIONARY, w.toBytes());
}

export function encodeValue(w: WireWriter, typeTag: TypeTag, value: SignalValue): void {
  switch (typeTag) {
    case TYPE_TAG.bool:
      w.u8(value ? 1 : 0);
      return;
    case TYPE_TAG.i64:
      if (typeof value === 'bigint') w.zigzagBig(value);
      else w.zigzagBig(BigInt(value as number));
      return;
    case TYPE_TAG.f64:
      w.f64(value as number);
      return;
    case TYPE_TAG.string:
      w.string(value as string);
      return;
    case TYPE_TAG['bool[]']: {
      const arr = value as boolean[];
      w.varint(arr.length);
      for (const b of arr) w.u8(b ? 1 : 0);
      return;
    }
    case TYPE_TAG['i64[]']: {
      const arr = value as (number | bigint)[];
      w.varint(arr.length);
      for (const n of arr) w.zigzagBig(typeof n === 'bigint' ? n : BigInt(n));
      return;
    }
    case TYPE_TAG['f64[]']: {
      const arr = value as number[];
      w.varint(arr.length);
      for (const n of arr) w.f64(n);
      return;
    }
    case TYPE_TAG['string[]']: {
      const arr = value as string[];
      w.varint(arr.length);
      for (const s of arr) w.string(s);
      return;
    }
    case TYPE_TAG.struct: {
      const s = value as { schemaId: number; bytes: Uint8Array };
      w.varint(s.schemaId).varint(s.bytes.length).bytes(s.bytes);
      return;
    }
  }
}

export function encodeFrame(frame: FrameInput): Uint8Array {
  const w = new WireWriter()
    .u8(FRAME_VERSION)
    .u8(
      (frame.keyframe ? FRAME_FLAG_KEYFRAME : 0) | (frame.fragment ? FRAME_FLAG_FRAGMENT : 0)
    )
    .u64(frame.cycleTimestampMicros)
    .u32(frame.sequence)
    .u16(frame.fragment?.index ?? 0)
    .u16(frame.fragment?.count ?? 1)
    .varint(frame.entries.length);
  for (const e of frame.entries) {
    w.varint(e.signalId).u8(e.typeTag);
    encodeValue(w, e.typeTag, e.value);
  }
  return encodeRecord(RECORD_TYPE.FRAME, w.toBytes());
}

export function encodeMarker(
  timestampMicros: number,
  kind: number,
  message: string,
  signalId = 0
): Uint8Array {
  const payload = new WireWriter()
    .zigzag(timestampMicros)
    .varint(kind)
    .string(message)
    .varint(signalId)
    .toBytes();
  return encodeRecord(RECORD_TYPE.MARKER, payload);
}

export function encodeMetadata(pairs: Record<string, string>): Uint8Array {
  const entries = Object.entries(pairs);
  const w = new WireWriter().varint(entries.length);
  for (const [k, v] of entries) w.string(k).string(v);
  return encodeRecord(RECORD_TYPE.METADATA, w.toBytes());
}

export function encodeStructSchema(schemaId: number, typeName: string, schema: string): Uint8Array {
  const payload = new WireWriter().varint(schemaId).string(typeName).string(schema).toBytes();
  return encodeRecord(RECORD_TYPE.STRUCT_SCHEMA, payload);
}

/** Wraps already-encoded records in a COMPRESSED_BLOCK using raw DEFLATE. */
export async function encodeCompressedBlock(records: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([records as BlobPart])
    .stream()
    .pipeThrough(new CompressionStream('deflate-raw'));
  const deflated = new Uint8Array(await new Response(stream).arrayBuffer());
  const payload = new WireWriter().varint(records.length).bytes(deflated).toBytes();
  return encodeRecord(RECORD_TYPE.COMPRESSED_BLOCK, payload);
}

export function concat(parts: Uint8Array[]): Uint8Array {
  const total = parts.reduce((n, p) => n + p.length, 0);
  const out = new Uint8Array(total);
  let offset = 0;
  for (const p of parts) {
    out.set(p, offset);
    offset += p.length;
  }
  return out;
}

/** Builds a `.athrp` file from already-encoded records. */
export function encodeReplay(records: Uint8Array): Uint8Array {
  const header = new WireWriter().bytes(new Uint8Array(REPLAY_MAGIC)).u16(REPLAY_FORMAT_VERSION);
  return concat([header.toBytes(), records]);
}
