package ca.frc6390.athena.arcp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.List;

/**
 * Buffers passed across JNI. This is a marshalling format between Java and the Rust server, not
 * the ARCP wire format: Rust turns these buffers into ARCP frames, replies and replay records.
 * The layout is documented in {@code athena-arcp/rust/crates/arcp-jni/src/marshal.rs}.
 *
 * <p>Writes go into a reusable direct buffer, so a cycle is one JNI call with no per-value
 * native calls. Value kinds reuse the ARCP typeTag numbers.</p>
 */
final class ArcpMarshal {

    static final int KIND_BOOL = 1;
    static final int KIND_I64 = 2;
    static final int KIND_F64 = 3;
    static final int KIND_STRING = 4;
    static final int KIND_BOOL_ARRAY = 5;
    static final int KIND_I64_ARRAY = 6;
    static final int KIND_F64_ARRAY = 7;
    static final int KIND_STRING_ARRAY = 8;
    static final int KIND_STRUCT = 9;

    static final int OP_WRITE = 1;
    static final int OP_ACTION_RUN = 2;
    static final int OP_ACTION_CANCEL = 3;
    static final int OP_BACKFILL = 4;

    static final int INITIAL_CAPACITY = 16 * 1024;

    private ArcpMarshal() {
    }

    /** Growable little-endian direct buffer for outgoing data. */
    static final class Out {
        private ByteBuffer buf;
        private int length;

        Out() {
            this(ByteBuffer.allocateDirect(INITIAL_CAPACITY));
        }

        Out(ByteBuffer buffer) {
            buf = buffer.order(ByteOrder.LITTLE_ENDIAN);
        }

        void reset() {
            length = 0;
        }

        int length() {
            return length;
        }

        ByteBuffer buffer() {
            return buf;
        }

        private void ensure(int extra) {
            if (buf.capacity() - length >= extra) {
                return;
            }
            int capacity = Math.max(buf.capacity() * 2, length + extra);
            ByteBuffer next = ByteBuffer.allocateDirect(capacity).order(ByteOrder.LITTLE_ENDIAN);
            ByteBuffer src = buf.duplicate();
            src.clear();
            src.limit(length);
            next.put(src);
            buf = next;
        }

        void u8(int v) {
            ensure(1);
            buf.put(length, (byte) v);
            length += 1;
        }

        void u32(int v) {
            ensure(4);
            buf.putInt(length, v);
            length += 4;
        }

        void u64(long v) {
            ensure(8);
            buf.putLong(length, v);
            length += 8;
        }

        void string(String s) {
            byte[] bytes = s.getBytes(StandardCharsets.UTF_8);
            u32(bytes.length);
            ensure(bytes.length);
            buf.put(length, bytes);
            length += bytes.length;
        }

        void patchU32(int at, int v) {
            buf.putInt(at, v);
        }

        /** Writes one value: kind byte, then the body. */
        void value(Object value) {
            if (value instanceof Boolean b) {
                u8(KIND_BOOL);
                u8(b ? 1 : 0);
            } else if (value instanceof Long l) {
                u8(KIND_I64);
                u64(l);
            } else if (value instanceof Integer i) {
                u8(KIND_I64);
                u64(i);
            } else if (value instanceof Double d) {
                u8(KIND_F64);
                u64(Double.doubleToRawLongBits(d));
            } else if (value instanceof String s) {
                u8(KIND_STRING);
                string(s);
            } else if (value instanceof boolean[] a) {
                u8(KIND_BOOL_ARRAY);
                u32(a.length);
                for (boolean b : a) {
                    u8(b ? 1 : 0);
                }
            } else if (value instanceof long[] a) {
                u8(KIND_I64_ARRAY);
                u32(a.length);
                for (long l : a) {
                    u64(l);
                }
            } else if (value instanceof double[] a) {
                u8(KIND_F64_ARRAY);
                u32(a.length);
                for (double d : a) {
                    u64(Double.doubleToRawLongBits(d));
                }
            } else if (value instanceof String[] a) {
                u8(KIND_STRING_ARRAY);
                u32(a.length);
                for (String s : a) {
                    string(s);
                }
            } else if (value instanceof ArcpStruct s) {
                byte[] bytes = s.bytes();
                u8(KIND_STRUCT);
                u64(s.schemaId());
                u32(bytes.length);
                ensure(bytes.length);
                buf.put(length, bytes);
                length += bytes.length;
            } else {
                throw new IllegalArgumentException("unsupported ARCP value type: "
                        + (value == null ? "null" : value.getClass().getName()));
            }
        }

        void dictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas) {
            reset();
            u32(entries.size());
            for (ArcpDictEntry e : entries) {
                u64(e.signalId());
                u8(e.typeTag());
                u8(e.role());
                u8(e.flags());
                u64(e.structSchemaId());
                string(e.path());
                string(e.unit());
                string(e.lockGroup());
            }
            u32(schemas.size());
            for (ArcpStructSchema s : schemas) {
                u64(s.id());
                string(s.typeName());
                string(s.schema());
            }
        }

        /** Writes an unsigned LEB128 varint. */
        void varint(long v) {
            while ((v & ~0x7FL) != 0) {
                u8((int) ((v & 0x7F) | 0x80));
                v >>>= 7;
            }
            u8((int) v);
        }

        /** Writes raw bytes. */
        void bytes(byte[] src, int offset, int count) {
            ensure(count);
            buf.put(length, src, offset, count);
            length += count;
        }

        void cycle(ArcpCycle cycle) {
            reset();
            int n = cycle.size();
            u32(n);
            for (int i = 0; i < n; i++) {
                u64(cycle.signalId(i));
                int kind = cycle.kind(i);
                switch (kind) {
                    case KIND_BOOL -> {
                        u8(KIND_BOOL);
                        u8(cycle.bits(i) != 0 ? 1 : 0);
                    }
                    case KIND_I64 -> {
                        u8(KIND_I64);
                        u64(cycle.bits(i));
                    }
                    case KIND_F64 -> {
                        u8(KIND_F64);
                        u64(cycle.bits(i));
                    }
                    default -> value(cycle.object(i));
                }
            }
        }
    }

    /** Reader for buffers written by the Rust side. */
    static final class In {
        private final ByteBuffer buf;
        private int pos;

        In(ByteBuffer buf) {
            this.buf = buf.duplicate().order(ByteOrder.LITTLE_ENDIAN);
        }

        int u8() {
            return buf.get(pos++) & 0xFF;
        }

        int u32() {
            int v = buf.getInt(pos);
            pos += 4;
            return v;
        }

        long u64() {
            long v = buf.getLong(pos);
            pos += 8;
            return v;
        }

        String string() {
            int n = u32();
            byte[] bytes = new byte[n];
            buf.get(pos, bytes);
            pos += n;
            return new String(bytes, StandardCharsets.UTF_8);
        }

        Object value() {
            int kind = u8();
            switch (kind) {
                case KIND_BOOL:
                    return u8() != 0;
                case KIND_I64:
                    return u64();
                case KIND_F64:
                    return Double.longBitsToDouble(u64());
                case KIND_STRING:
                    return string();
                case KIND_BOOL_ARRAY: {
                    int n = u32();
                    boolean[] a = new boolean[n];
                    for (int i = 0; i < n; i++) {
                        a[i] = u8() != 0;
                    }
                    return a;
                }
                case KIND_I64_ARRAY: {
                    int n = u32();
                    long[] a = new long[n];
                    for (int i = 0; i < n; i++) {
                        a[i] = u64();
                    }
                    return a;
                }
                case KIND_F64_ARRAY: {
                    int n = u32();
                    double[] a = new double[n];
                    for (int i = 0; i < n; i++) {
                        a[i] = Double.longBitsToDouble(u64());
                    }
                    return a;
                }
                case KIND_STRING_ARRAY: {
                    int n = u32();
                    String[] a = new String[n];
                    for (int i = 0; i < n; i++) {
                        a[i] = string();
                    }
                    return a;
                }
                case KIND_STRUCT: {
                    long schemaId = u64();
                    int n = u32();
                    byte[] bytes = new byte[n];
                    buf.get(pos, bytes);
                    pos += n;
                    return new ArcpStruct(schemaId, bytes);
                }
                default:
                    throw new IllegalStateException("unknown value kind " + kind + " from native server");
            }
        }
    }
}
