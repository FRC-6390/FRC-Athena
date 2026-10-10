package ca.frc6390.athena.runtime.wire;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * Growable little-endian byte writer for the ARCP v2 / replay wire format.
 *
 * <p>See {@code docs/2027-offseason-design.md}, appendix E1-W. Instances are reusable through
 * {@link #clear()} so per-cycle encoding does not allocate once the buffer has grown.</p>
 */
public final class WireWriter {
    private byte[] buffer;
    private int size;

    public WireWriter() {
        this(256);
    }

    public WireWriter(int initialCapacity) {
        buffer = new byte[Math.max(16, initialCapacity)];
    }

    public WireWriter clear() {
        size = 0;
        return this;
    }

    public int size() {
        return size;
    }

    /** Returns the internal buffer; only the first {@link #size()} bytes are valid. */
    public byte[] buffer() {
        return buffer;
    }

    public byte[] toByteArray() {
        return Arrays.copyOf(buffer, size);
    }

    public WireWriter u8(int value) {
        ensure(1);
        buffer[size++] = (byte) value;
        return this;
    }

    public WireWriter u16(int value) {
        ensure(2);
        buffer[size++] = (byte) value;
        buffer[size++] = (byte) (value >>> 8);
        return this;
    }

    public WireWriter u32(long value) {
        ensure(4);
        for (int i = 0; i < 4; i++) {
            buffer[size++] = (byte) (value >>> (8 * i));
        }
        return this;
    }

    public WireWriter u64(long value) {
        ensure(8);
        for (int i = 0; i < 8; i++) {
            buffer[size++] = (byte) (value >>> (8 * i));
        }
        return this;
    }

    public WireWriter f64(double value) {
        return u64(Double.doubleToRawLongBits(value));
    }

    /** Unsigned LEB128. Negative values are encoded as their unsigned 64-bit pattern. */
    public WireWriter varint(long value) {
        ensure(10);
        long remaining = value;
        while ((remaining & ~0x7FL) != 0) {
            buffer[size++] = (byte) ((remaining & 0x7F) | 0x80);
            remaining >>>= 7;
        }
        buffer[size++] = (byte) remaining;
        return this;
    }

    public WireWriter zigzag(long value) {
        return varint((value << 1) ^ (value >> 63));
    }

    public WireWriter string(String value) {
        byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
        varint(bytes.length);
        return bytes(bytes, 0, bytes.length);
    }

    public WireWriter bytes(byte[] source, int offset, int length) {
        ensure(length);
        System.arraycopy(source, offset, buffer, size, length);
        size += length;
        return this;
    }

    /** Appends {@code payload} wrapped in a record envelope: type, varint length, bytes. */
    public WireWriter record(int recordType, WireWriter payload) {
        u8(recordType);
        varint(payload.size());
        return bytes(payload.buffer(), 0, payload.size());
    }

    private void ensure(int additional) {
        int required = size + additional;
        if (required > buffer.length) {
            buffer = Arrays.copyOf(buffer, Math.max(required, buffer.length * 2));
        }
    }
}
