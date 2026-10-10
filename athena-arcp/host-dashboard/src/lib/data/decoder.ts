/**
 * Decoder for the ARCP v2 / replay byte format (appendix E1-W). Pure TypeScript; the only platform
 * dependency is `DecompressionStream('deflate-raw')` for COMPRESSED_BLOCK records.
 */
import { describePath } from './path';
import { WireError, WireReader } from './wire-io';
import {
  MARKER_KIND,
  ROLE_BY_CODE,
  TYPE_TAG,
  type DecodedRecord,
  type Frame,
  type FrameEntry,
  type I64Value,
  type Marker,
  type SignalDescriptor,
  type SignalType,
  type SignalValue,
  type StructSchema,
  type TypeTag
} from './types';

export const RECORD_TYPE = {
  DICTIONARY: 0x01,
  FRAME: 0x02,
  MARKER: 0x03,
  METADATA: 0x04,
  STRUCT_SCHEMA: 0x05,
  COMPRESSED_BLOCK: 0x10
} as const;

export const REPLAY_MAGIC = [0x41, 0x54, 0x48, 0x52, 0x50, 0x00]; // "ATHRP\0"
export const REPLAY_FORMAT_VERSION = 1;
export const FRAME_VERSION = 2;
export const FRAME_FLAG_KEYFRAME = 0x01;
export const FRAME_FLAG_FRAGMENT = 0x02;
export const DICTIONARY_FLAG_WRITABLE = 0x01;
export const DICTIONARY_FLAG_PERSISTED = 0x02;

/** Upper bound on nested COMPRESSED_BLOCK expansion; real writers never nest. */
const MAX_NESTING = 2;

/** Per-decode state: struct schemas seen so far, used to name struct signal types. */
export type DecodeContext = {
  structSchemas: Map<number, StructSchema>;
};

export function createDecodeContext(): DecodeContext {
  return { structSchemas: new Map() };
}

export function markerKindLabel(kind: number): string {
  switch (kind) {
    case MARKER_KIND.BOOT:
      return 'Boot';
    case MARKER_KIND.MATCH_IDENTIFIED:
      return 'Match identified';
    case MARKER_KIND.AUTO_START:
      return 'Auto start';
    case MARKER_KIND.TELEOP_START:
      return 'Teleop start';
    case MARKER_KIND.MATCH_END:
      return 'Match end';
    case MARKER_KIND.DISABLE:
      return 'Disable';
    case MARKER_KIND.FAULT:
      return 'Fault';
    case MARKER_KIND.STAGE_CHANGE:
      return 'Stage change';
    case MARKER_KIND.CHECK:
      return 'Check';
    case MARKER_KIND.OVERRIDE:
      return 'Override';
    case MARKER_KIND.PERSISTED_WRITE:
      return 'Persisted write';
    case MARKER_KIND.RECORD:
      return 'Record';
    case MARKER_KIND.TEST_START:
      return 'Test start';
    default:
      return kind >= MARKER_KIND.USER_BASE ? `User ${kind}` : `Unknown ${kind}`;
  }
}

/** Maps a numeric type tag to a SignalType. Structs need the schema table to get their type name. */
export function signalTypeForTag(
  tag: TypeTag,
  structSchemaId: number,
  ctx: DecodeContext
): SignalType {
  switch (tag) {
    case TYPE_TAG.bool:
      return 'bool';
    case TYPE_TAG.i64:
      return 'i64';
    case TYPE_TAG.f64:
      return 'f64';
    case TYPE_TAG.string:
      return 'string';
    case TYPE_TAG['bool[]']:
      return 'bool[]';
    case TYPE_TAG['i64[]']:
      return 'i64[]';
    case TYPE_TAG['f64[]']:
      return 'f64[]';
    case TYPE_TAG['string[]']:
      return 'string[]';
    case TYPE_TAG.struct: {
      const schema = ctx.structSchemas.get(structSchemaId);
      return `struct:${schema ? schema.typeName : `#${structSchemaId}`}`;
    }
    default:
      throw new WireError(`unknown type tag ${tag}`, -1);
  }
}

function readTypeTag(r: WireReader): TypeTag {
  const tag = r.u8();
  if (tag < 1 || tag > 9) {
    throw new WireError(`unknown type tag ${tag}`, r.offset - 1);
  }
  return tag as TypeTag;
}

const MAX_SAFE = BigInt(Number.MAX_SAFE_INTEGER);
const MIN_SAFE = BigInt(Number.MIN_SAFE_INTEGER);

/** Exact i64s become numbers; anything outside the safe range stays a bigint (no silent rounding). */
export function normalizeI64(value: bigint): I64Value {
  return value >= MIN_SAFE && value <= MAX_SAFE ? Number(value) : value;
}

/** Reads one typed value (the part after `varint signalId, u8 typeTag`). */
export function readValue(r: WireReader, tag: TypeTag): SignalValue {
  switch (tag) {
    case TYPE_TAG.bool:
      return r.u8() !== 0;
    case TYPE_TAG.i64:
      return normalizeI64(r.zigzagBig());
    case TYPE_TAG.f64:
      return r.f64();
    case TYPE_TAG.string:
      return r.string();
    case TYPE_TAG['bool[]']: {
      const n = r.count();
      const out: boolean[] = [];
      for (let i = 0; i < n; i++) out.push(r.u8() !== 0);
      return out;
    }
    case TYPE_TAG['i64[]']: {
      const n = r.count();
      const out: I64Value[] = [];
      for (let i = 0; i < n; i++) out.push(normalizeI64(r.zigzagBig()));
      return out;
    }
    case TYPE_TAG['f64[]']: {
      const n = r.varint();
      if (n * 8 > r.remaining) {
        throw new WireError(`f64[] length ${n} exceeds remaining data`, r.offset);
      }
      const out: number[] = [];
      for (let i = 0; i < n; i++) out.push(r.f64());
      return out;
    }
    case TYPE_TAG['string[]']: {
      const n = r.count();
      const out: string[] = [];
      for (let i = 0; i < n; i++) out.push(r.string());
      return out;
    }
    case TYPE_TAG.struct: {
      const schemaId = r.varint();
      const length = r.varint();
      return { schemaId, bytes: copyBytes(r.bytesOf(length)) };
    }
  }
}

function copyBytes(view: Uint8Array): Uint8Array {
  return new Uint8Array(view);
}

export function decodeFrame(payload: Uint8Array): Frame {
  const r = new WireReader(payload);
  const version = r.u8();
  if (version !== FRAME_VERSION) {
    throw new WireError(`unsupported frame version ${version}`, 0);
  }
  const flags = r.u8();
  const cycleTimestampMicros = r.u64();
  const sequence = r.u32();
  const fragmentIndex = r.u16();
  const fragmentCount = r.u16();
  const entryCount = r.count();
  const entries: FrameEntry[] = [];
  for (let i = 0; i < entryCount; i++) {
    const signalId = r.varint();
    const typeTag = readTypeTag(r);
    entries.push({ signalId, typeTag, value: readValue(r, typeTag) });
  }
  return {
    version,
    keyframe: (flags & FRAME_FLAG_KEYFRAME) !== 0,
    fragmented: (flags & FRAME_FLAG_FRAGMENT) !== 0,
    cycleTimestampMicros,
    sequence,
    fragmentIndex,
    fragmentCount,
    entries
  };
}

export function decodeDictionary(
  payload: Uint8Array,
  ctx: DecodeContext = createDecodeContext()
): SignalDescriptor[] {
  const r = new WireReader(payload);
  const count = r.count();
  const entries: SignalDescriptor[] = [];
  for (let i = 0; i < count; i++) {
    const signalId = r.varint();
    const path = r.string();
    const typeTag = readTypeTag(r);
    const unit = r.string();
    const roleCode = r.u8();
    const flags = r.u8();
    const lockGroup = r.string();
    const structSchemaId = r.varint();
    const type = signalTypeForTag(typeTag, structSchemaId, ctx);
    entries.push({
      signalId,
      ...describePath(path),
      type,
      typeTag,
      unit,
      role: ROLE_BY_CODE[roleCode] ?? 'unspecified',
      writable: (flags & DICTIONARY_FLAG_WRITABLE) !== 0,
      persisted: (flags & DICTIONARY_FLAG_PERSISTED) !== 0,
      lockGroup,
      structSchemaId
    });
  }
  return entries;
}

export function decodeMarker(payload: Uint8Array): Marker {
  const r = new WireReader(payload);
  const timestampMicros = r.zigzag();
  const kind = r.varint();
  const message = r.string();
  const signalId = r.varint();
  return {
    timestampMicros,
    kind,
    label: markerKindLabel(kind),
    message,
    signalId: signalId === 0 ? null : signalId
  };
}

export function decodeMetadata(payload: Uint8Array): Record<string, string> {
  const r = new WireReader(payload);
  const count = r.count();
  const pairs: Record<string, string> = {};
  for (let i = 0; i < count; i++) {
    const key = r.string();
    pairs[key] = r.string();
  }
  return pairs;
}

export function decodeStructSchema(payload: Uint8Array): StructSchema {
  const r = new WireReader(payload);
  return {
    schemaId: r.varint(),
    typeName: r.string(),
    wpilibSchema: r.string()
  };
}

/** Inflates a raw DEFLATE stream (RFC 1951, no zlib header). */
export async function inflateRaw(bytes: Uint8Array): Promise<Uint8Array> {
  if (typeof DecompressionStream === 'undefined') {
    throw new Error('DecompressionStream("deflate-raw") is not available in this runtime');
  }
  const stream = new Blob([bytes as BlobPart])
    .stream()
    .pipeThrough(new DecompressionStream('deflate-raw'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

/**
 * Decodes a sequence of record envelopes. COMPRESSED_BLOCK payloads are inflated and their records are
 * spliced into the output in place. Unknown record types are skipped (forward compatibility).
 */
export async function decodeRecords(
  bytes: Uint8Array,
  ctx: DecodeContext = createDecodeContext(),
  depth = 0
): Promise<DecodedRecord[]> {
  const out: DecodedRecord[] = [];
  const r = new WireReader(bytes);
  while (r.remaining > 0) {
    const type = r.u8();
    const length = r.varint();
    const payload = r.bytesOf(length);
    switch (type) {
      case RECORD_TYPE.DICTIONARY:
        out.push({ kind: 'dictionary', entries: decodeDictionary(payload, ctx) });
        break;
      case RECORD_TYPE.FRAME:
        out.push({ kind: 'frame', frame: decodeFrame(payload) });
        break;
      case RECORD_TYPE.MARKER:
        out.push({ kind: 'marker', marker: decodeMarker(payload) });
        break;
      case RECORD_TYPE.METADATA:
        out.push({ kind: 'metadata', pairs: decodeMetadata(payload) });
        break;
      case RECORD_TYPE.STRUCT_SCHEMA: {
        const schema = decodeStructSchema(payload);
        ctx.structSchemas.set(schema.schemaId, schema);
        out.push({ kind: 'struct_schema', schema });
        break;
      }
      case RECORD_TYPE.COMPRESSED_BLOCK: {
        if (depth >= MAX_NESTING) {
          throw new WireError('compressed blocks nested too deeply', r.offset);
        }
        const inner = new WireReader(payload);
        const uncompressedLength = inner.varint();
        const inflated = await inflateRaw(payload.subarray(inner.offset));
        if (inflated.length !== uncompressedLength) {
          throw new WireError(
            `compressed block length ${inflated.length} != declared ${uncompressedLength}`,
            r.offset
          );
        }
        out.push(...(await decodeRecords(inflated, ctx, depth + 1)));
        break;
      }
      default:
        // Unknown record type: skipped by its declared length.
        break;
    }
  }
  return out;
}

/** Decodes a `.athrp` replay file: magic, u16 format version, then records. */
export async function decodeReplay(bytes: Uint8Array): Promise<DecodedRecord[]> {
  if (bytes.length < REPLAY_MAGIC.length + 2) {
    throw new WireError('replay file too short', 0);
  }
  for (let i = 0; i < REPLAY_MAGIC.length; i++) {
    if (bytes[i] !== REPLAY_MAGIC[i]) {
      throw new WireError('bad replay magic', i);
    }
  }
  const r = new WireReader(bytes, REPLAY_MAGIC.length);
  const formatVersion = r.u16();
  if (formatVersion !== REPLAY_FORMAT_VERSION) {
    throw new WireError(`unsupported replay format version ${formatVersion}`, REPLAY_MAGIC.length);
  }
  return decodeRecords(bytes.subarray(r.offset));
}
