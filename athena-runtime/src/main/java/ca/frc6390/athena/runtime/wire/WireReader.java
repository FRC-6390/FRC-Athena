package ca.frc6390.athena.runtime.wire;

import java.nio.charset.StandardCharsets;

/** Little-endian reader for the ARCP v2 / replay wire format (appendix E1-W). */
public final class WireReader {
    private final byte[] data;
    private final int end;
    private int position;

    public WireReader(byte[] data) {
        this(data, 0, data.length);
    }

    public WireReader(byte[] data, int offset, int length) {
        if (offset < 0 || length < 0 || offset + length > data.length) {
            throw new IllegalArgumentException("Invalid wire slice.");
        }
        this.data = data;
        this.position = offset;
        this.end = offset + length;
    }

    public int position() {
        return position;
    }

    public int remaining() {
        return end - position;
    }

    public boolean hasRemaining() {
        return position < end;
    }

    public int u8() {
        require(1);
        return data[position++] & 0xFF;
    }

    public int u16() {
        require(2);
        int value = (data[position] & 0xFF) | ((data[position + 1] & 0xFF) << 8);
        position += 2;
        return value;
    }

    public long u32() {
        require(4);
        long value = 0;
        for (int i = 0; i < 4; i++) {
            value |= (long) (data[position++] & 0xFF) << (8 * i);
        }
        return value;
    }

    public long u64() {
        require(8);
        long value = 0;
        for (int i = 0; i < 8; i++) {
            value |= (long) (data[position++] & 0xFF) << (8 * i);
        }
        return value;
    }

    public double f64() {
        return Double.longBitsToDouble(u64());
    }

    public long varint() {
        long value = 0;
        int shift = 0;
        while (true) {
            if (shift > 63) {
                throw new IllegalStateException("Varint is too long.");
            }
            int next = u8();
            value |= (long) (next & 0x7F) << shift;
            if ((next & 0x80) == 0) {
                return value;
            }
            shift += 7;
        }
    }

    public long zigzag() {
        long raw = varint();
        return (raw >>> 1) ^ -(raw & 1);
    }

    public String string() {
        int length = checkedLength(varint());
        require(length);
        String value = new String(data, position, length, StandardCharsets.UTF_8);
        position += length;
        return value;
    }

    public byte[] bytes(int length) {
        require(length);
        byte[] copy = new byte[length];
        System.arraycopy(data, position, copy, 0, length);
        position += length;
        return copy;
    }

    /** Reads one record envelope; the returned reader covers only that record's payload. */
    public Record record() {
        int type = u8();
        int length = checkedLength(varint());
        require(length);
        WireReader payload = new WireReader(data, position, length);
        position += length;
        return new Record(type, payload);
    }

    public record Record(int type, WireReader payload) {
    }

    private int checkedLength(long length) {
        if (length < 0 || length > Integer.MAX_VALUE) {
            throw new IllegalStateException("Invalid wire length " + length + ".");
        }
        return (int) length;
    }

    private void require(int count) {
        if (count < 0 || position + count > end) {
            throw new IllegalStateException("Truncated wire data.");
        }
    }
}
