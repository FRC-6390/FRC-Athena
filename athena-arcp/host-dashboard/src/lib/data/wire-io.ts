/**
 * Little-endian primitive reader and writer for the ARCP v2 / replay format (E1-W).
 * Mirrors athena-runtime `ca.frc6390.athena.runtime.wire.WireReader` / `WireWriter`.
 *
 * 64-bit integers are represented as JavaScript numbers. Timestamps (micros) and sequence numbers stay
 * well inside the 2^53 safe range; i64 signal values beyond that range lose precision.
 */

export class WireError extends Error {
  constructor(
    message: string,
    readonly offset: number
  ) {
    super(`${message} (at byte ${offset})`);
    this.name = 'WireError';
  }
}

const TWO_32 = 0x1_0000_0000;

export class WireReader {
  private readonly view: DataView;
  offset: number;

  constructor(
    readonly bytes: Uint8Array,
    start = 0,
    readonly end = bytes.byteLength
  ) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.offset = start;
  }

  get remaining(): number {
    return this.end - this.offset;
  }

  private need(count: number): void {
    if (count < 0 || this.offset + count > this.end) {
      throw new WireError(`unexpected end of data, need ${count} bytes`, this.offset);
    }
  }

  u8(): number {
    this.need(1);
    return this.view.getUint8(this.offset++);
  }

  u16(): number {
    this.need(2);
    const value = this.view.getUint16(this.offset, true);
    this.offset += 2;
    return value;
  }

  u32(): number {
    this.need(4);
    const value = this.view.getUint32(this.offset, true);
    this.offset += 4;
    return value;
  }

  u64(): number {
    this.need(8);
    const value = this.view.getBigUint64(this.offset, true);
    this.offset += 8;
    return Number(value);
  }

  f64(): number {
    this.need(8);
    const value = this.view.getFloat64(this.offset, true);
    this.offset += 8;
    return value;
  }

  /** Unsigned LEB128. Values up to 2^53 are exact. */
  varint(): number {
    let result = 0;
    let scale = 1;
    for (let i = 0; i < 10; i++) {
      const byte = this.u8();
      result += (byte & 0x7f) * scale;
      if ((byte & 0x80) === 0) {
        return result;
      }
      scale *= 128;
    }
    throw new WireError('varint longer than 10 bytes', this.offset);
  }

  /** Signed zigzag varint. */
  zigzag(): number {
    const raw = this.varint();
    return raw % 2 === 0 ? raw / 2 : -(raw + 1) / 2;
  }

  /** `varint byteLength` followed by UTF-8 bytes. */
  string(): string {
    const length = this.varint();
    const raw = this.bytesOf(length);
    return new TextDecoder('utf-8', { fatal: true }).decode(raw);
  }

  /** Returns a view (not a copy) of the next `count` bytes. */
  bytesOf(count: number): Uint8Array {
    this.need(count);
    const slice = this.bytes.subarray(this.offset, this.offset + count);
    this.offset += count;
    return slice;
  }

  /** Reads a varint element count and guards it against the bytes left (each element needs >= 1 byte). */
  count(): number {
    const count = this.varint();
    if (count > this.remaining) {
      throw new WireError(`count ${count} exceeds remaining data`, this.offset);
    }
    return count;
  }
}

export class WireWriter {
  private buffer: Uint8Array;
  private view: DataView;
  size = 0;

  constructor(initialCapacity = 256) {
    this.buffer = new Uint8Array(Math.max(16, initialCapacity));
    this.view = new DataView(this.buffer.buffer);
  }

  toBytes(): Uint8Array {
    return this.buffer.slice(0, this.size);
  }

  private ensure(extra: number): void {
    const required = this.size + extra;
    if (required <= this.buffer.length) return;
    const next = new Uint8Array(Math.max(required, this.buffer.length * 2));
    next.set(this.buffer.subarray(0, this.size));
    this.buffer = next;
    this.view = new DataView(next.buffer);
  }

  u8(value: number): this {
    this.ensure(1);
    this.view.setUint8(this.size++, value & 0xff);
    return this;
  }

  u16(value: number): this {
    this.ensure(2);
    this.view.setUint16(this.size, value, true);
    this.size += 2;
    return this;
  }

  u32(value: number): this {
    this.ensure(4);
    this.view.setUint32(this.size, value >>> 0, true);
    this.size += 4;
    return this;
  }

  u64(value: number): this {
    this.ensure(8);
    const high = Math.floor(value / TWO_32);
    const low = value - high * TWO_32;
    this.view.setUint32(this.size, low, true);
    this.view.setUint32(this.size + 4, high, true);
    this.size += 8;
    return this;
  }

  f64(value: number): this {
    this.ensure(8);
    this.view.setFloat64(this.size, value, true);
    this.size += 8;
    return this;
  }

  varint(value: number): this {
    if (!Number.isInteger(value) || value < 0) {
      throw new RangeError(`varint must be a non-negative integer: ${value}`);
    }
    let remaining = value;
    while (remaining >= 0x80) {
      this.u8((remaining % 0x80) | 0x80);
      remaining = Math.floor(remaining / 0x80);
    }
    return this.u8(remaining);
  }

  zigzag(value: number): this {
    if (!Number.isInteger(value)) {
      throw new RangeError(`zigzag value must be an integer: ${value}`);
    }
    return this.varint(value >= 0 ? value * 2 : -value * 2 - 1);
  }

  string(value: string): this {
    const bytes = new TextEncoder().encode(value);
    this.varint(bytes.length);
    return this.bytes(bytes);
  }

  bytes(source: Uint8Array): this {
    this.ensure(source.length);
    this.buffer.set(source, this.size);
    this.size += source.length;
    return this;
  }

  /** Appends a record envelope: `u8 recordType, varint payloadLength, payload`. */
  record(recordType: number, payload: Uint8Array): this {
    this.u8(recordType);
    this.varint(payload.length);
    return this.bytes(payload);
  }
}
