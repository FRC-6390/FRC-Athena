package ca.frc6390.athena.arcp.v2;

import ca.frc6390.athena.arcp.v2.ArcpMessages.ClientRequest;
import ca.frc6390.athena.arcp.v2.ArcpMessages.FileInfo;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ReplyData;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ServerMessage;
import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * ARCP v2 codec in Java: typed values, dictionary and schema bodies, frames, markers and control
 * messages. Byte-compatible with {@code arcp_core::v2} in Rust; both are checked against the golden
 * vectors in {@code docs/wire-vectors}. Primitive encoding is delegated to the reference
 * {@link WireWriter} and {@link WireReader} in {@code athena-runtime}.
 */
public final class ArcpCodec {

    public static final int PROTOCOL_VERSION = 2;
    public static final int MAX_CONTROL_BODY = 1 << 20;
    public static final int MAX_FILE_CHUNK = 60_000;

    // Type tags (appendix E1-W).
    public static final int TYPE_BOOL = 1;
    public static final int TYPE_I64 = 2;
    public static final int TYPE_F64 = 3;
    public static final int TYPE_STRING = 4;
    public static final int TYPE_BOOL_ARRAY = 5;
    public static final int TYPE_I64_ARRAY = 6;
    public static final int TYPE_F64_ARRAY = 7;
    public static final int TYPE_STRING_ARRAY = 8;
    public static final int TYPE_STRUCT = 9;

    // Marker kinds.
    public static final int MARKER_BOOT = Wire.MARKER_BOOT;
    public static final int MARKER_MATCH_IDENTIFIED = Wire.MARKER_MATCH_IDENTIFIED;
    public static final int MARKER_AUTO_START = Wire.MARKER_AUTO_START;
    public static final int MARKER_TELEOP_START = Wire.MARKER_TELEOP_START;
    public static final int MARKER_MATCH_END = Wire.MARKER_MATCH_END;
    public static final int MARKER_DISABLE = Wire.MARKER_DISABLE;
    public static final int MARKER_FAULT = Wire.MARKER_FAULT;
    public static final int MARKER_STAGE_CHANGE = Wire.MARKER_STAGE_CHANGE;
    public static final int MARKER_CHECK = Wire.MARKER_CHECK;
    public static final int MARKER_OVERRIDE = Wire.MARKER_OVERRIDE;
    public static final int MARKER_PERSISTED_WRITE = Wire.MARKER_PERSISTED_WRITE;
    public static final int MARKER_RECORD = Wire.MARKER_RECORD;
    public static final int MARKER_TEST_START = Wire.MARKER_TEST_START;
    public static final int MARKER_USER_BASE = Wire.MARKER_USER_BASE;

    // Control message kinds.
    static final int KIND_HELLO = 0x01;
    static final int KIND_SUBSCRIBE = 0x10;
    static final int KIND_UNSUBSCRIBE = 0x11;
    static final int KIND_WRITE = 0x20;
    static final int KIND_ACTION_RUN = 0x21;
    static final int KIND_ACTION_CANCEL = 0x22;
    static final int KIND_TIME_PING = 0x30;
    static final int KIND_SCHEMA_REQUEST = 0x40;
    static final int KIND_FILE_LIST = 0x50;
    static final int KIND_FILE_DOWNLOAD = 0x51;
    static final int KIND_FILE_DELETE = 0x52;
    static final int KIND_REPLY = 0x80;
    static final int KIND_SCHEMA = 0x81;
    static final int KIND_TIME_PONG = 0x82;
    static final int KIND_WRITE_REPLY = 0x83;

    private static final int RATE_ON_CHANGE = 0;
    private static final int RATE_HZ = 1;

    private ArcpCodec() {
    }

    // ---- values ------------------------------------------------------------------------------

    /** @return the wire type tag for a supported Java value, or -1 when unsupported */
    public static int tagOf(Object value) {
        if (value instanceof Boolean) {
            return TYPE_BOOL;
        }
        if (value instanceof Long || value instanceof Integer) {
            return TYPE_I64;
        }
        if (value instanceof Double || value instanceof Float) {
            return TYPE_F64;
        }
        if (value instanceof String) {
            return TYPE_STRING;
        }
        if (value instanceof boolean[]) {
            return TYPE_BOOL_ARRAY;
        }
        if (value instanceof long[]) {
            return TYPE_I64_ARRAY;
        }
        if (value instanceof double[]) {
            return TYPE_F64_ARRAY;
        }
        if (value instanceof String[]) {
            return TYPE_STRING_ARRAY;
        }
        if (value instanceof ArcpStruct) {
            return TYPE_STRUCT;
        }
        return -1;
    }

    /** Writes {@code u8 typeTag} followed by the body. */
    public static void writeValue(WireWriter w, Object value) {
        int tag = tagOf(value);
        if (tag < 0) {
            throw new IllegalArgumentException("unsupported ARCP value type: " + value.getClass().getName());
        }
        w.u8(tag);
        writeBody(w, tag, value);
    }

    private static void writeBody(WireWriter w, int tag, Object value) {
        switch (tag) {
            case TYPE_BOOL -> w.u8((Boolean) value ? 1 : 0);
            case TYPE_I64 -> w.zigzag(((Number) value).longValue());
            case TYPE_F64 -> w.f64(((Number) value).doubleValue());
            case TYPE_STRING -> w.string((String) value);
            case TYPE_BOOL_ARRAY -> {
                boolean[] v = (boolean[]) value;
                w.varint(v.length);
                for (boolean b : v) {
                    w.u8(b ? 1 : 0);
                }
            }
            case TYPE_I64_ARRAY -> {
                long[] v = (long[]) value;
                w.varint(v.length);
                for (long n : v) {
                    w.zigzag(n);
                }
            }
            case TYPE_F64_ARRAY -> {
                double[] v = (double[]) value;
                w.varint(v.length);
                for (double n : v) {
                    w.f64(n);
                }
            }
            case TYPE_STRING_ARRAY -> {
                String[] v = (String[]) value;
                w.varint(v.length);
                for (String s : v) {
                    w.string(s);
                }
            }
            case TYPE_STRUCT -> {
                ArcpStruct s = (ArcpStruct) value;
                w.varint(s.schemaId());
                byte[] bytes = s.bytes();
                w.varint(bytes.length);
                putBytes(w, bytes);
            }
            default -> throw new IllegalArgumentException("unknown tag " + tag);
        }
    }

    /** Reads {@code u8 typeTag} followed by the body. */
    public static Object readValue(WireReader r) {
        return readBody(r, r.u8());
    }

    private static Object readBody(WireReader r, int tag) {
        return switch (tag) {
            case TYPE_BOOL -> boolValue(r.u8());
            case TYPE_I64 -> r.zigzag();
            case TYPE_F64 -> r.f64();
            case TYPE_STRING -> r.string();
            case TYPE_BOOL_ARRAY -> {
                int n = count(r, 1);
                boolean[] out = new boolean[n];
                for (int i = 0; i < n; i++) {
                    out[i] = boolValue(r.u8());
                }
                yield out;
            }
            case TYPE_I64_ARRAY -> {
                int n = count(r, 1);
                long[] out = new long[n];
                for (int i = 0; i < n; i++) {
                    out[i] = r.zigzag();
                }
                yield out;
            }
            case TYPE_F64_ARRAY -> {
                int n = count(r, 8);
                double[] out = new double[n];
                for (int i = 0; i < n; i++) {
                    out[i] = r.f64();
                }
                yield out;
            }
            case TYPE_STRING_ARRAY -> {
                int n = count(r, 1);
                String[] out = new String[n];
                for (int i = 0; i < n; i++) {
                    out[i] = r.string();
                }
                yield out;
            }
            case TYPE_STRUCT -> {
                long schemaId = r.varint();
                int len = length(r);
                yield new ArcpStruct(schemaId, r.bytes(len));
            }
            default -> throw new IllegalStateException("unknown type tag " + tag);
        };
    }

    private static boolean boolValue(int raw) {
        if (raw == 0) {
            return false;
        }
        if (raw == 1) {
            return true;
        }
        throw new IllegalStateException("invalid bool byte " + raw);
    }

    /** Equality by value, comparing floating point by bit pattern. */
    public static boolean sameValue(Object a, Object b) {
        if (a instanceof double[] x && b instanceof double[] y) {
            if (x.length != y.length) {
                return false;
            }
            for (int i = 0; i < x.length; i++) {
                if (Double.doubleToRawLongBits(x[i]) != Double.doubleToRawLongBits(y[i])) {
                    return false;
                }
            }
            return true;
        }
        if (a instanceof Double x && b instanceof Double y) {
            return Double.doubleToRawLongBits(x) == Double.doubleToRawLongBits(y);
        }
        if (a instanceof boolean[] x && b instanceof boolean[] y) {
            return Arrays.equals(x, y);
        }
        if (a instanceof long[] x && b instanceof long[] y) {
            return Arrays.equals(x, y);
        }
        if (a instanceof String[] x && b instanceof String[] y) {
            return Arrays.equals(x, y);
        }
        return a == null ? b == null : a.equals(b);
    }

    // ---- dictionary and schema ---------------------------------------------------------------

    /** Writes a dictionary body: varint count, then entries. */
    public static void writeDictionary(WireWriter w, List<ArcpDictEntry> entries) {
        w.varint(entries.size());
        for (ArcpDictEntry e : entries) {
            w.varint(e.signalId())
                    .string(e.path())
                    .u8(e.typeTag())
                    .string(e.unit())
                    .u8(e.role())
                    .u8(e.flags())
                    .string(e.lockGroup())
                    .varint(e.structSchemaId());
        }
    }

    public static List<ArcpDictEntry> readDictionary(WireReader r) {
        int n = count(r, 8);
        List<ArcpDictEntry> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            out.add(new ArcpDictEntry(
                    r.varint(), r.string(), r.u8(), r.string(), r.u8(), r.u8(), r.string(), r.varint()));
        }
        return out;
    }

    public static void writeStructSchema(WireWriter w, ArcpStructSchema s) {
        w.varint(s.id()).string(s.typeName()).string(s.schema());
    }

    public static ArcpStructSchema readStructSchema(WireReader r) {
        return new ArcpStructSchema(r.varint(), r.string(), r.string());
    }

    /** A complete DICTIONARY record (envelope included). */
    public static byte[] dictionaryRecord(List<ArcpDictEntry> entries) {
        WireWriter payload = new WireWriter();
        writeDictionary(payload, entries);
        return envelope(Wire.RECORD_DICTIONARY, payload);
    }

    /** A complete STRUCT_SCHEMA record (envelope included). */
    public static byte[] structSchemaRecord(ArcpStructSchema schema) {
        WireWriter payload = new WireWriter();
        writeStructSchema(payload, schema);
        return envelope(Wire.RECORD_STRUCT_SCHEMA, payload);
    }

    /** A complete MARKER record (envelope included). */
    public static byte[] markerRecord(long timestampMicros, int kind, String message, long signalId) {
        WireWriter payload = new WireWriter();
        Wire.marker(payload, timestampMicros, kind, message, signalId);
        return envelope(Wire.RECORD_MARKER, payload);
    }

    // ---- frames ------------------------------------------------------------------------------

    /**
     * Writes a complete FRAME record (envelope included) for one fragment.
     *
     * @param entries entries already encoded with {@link #writeEntry}
     */
    public static byte[] frameRecord(boolean keyframe, boolean fragmented, long timestampMicros, long sequence,
            int fragmentIndex, int fragmentCount, List<byte[]> encodedEntries) {
        WireWriter payload = new WireWriter(64);
        int flags = (keyframe ? Wire.FRAME_FLAG_KEYFRAME : 0) | (fragmented ? Wire.FRAME_FLAG_FRAGMENT : 0);
        payload.u8(Wire.FRAME_VERSION)
                .u8(flags)
                .u64(timestampMicros)
                .u32(sequence)
                .u16(fragmented ? fragmentIndex : 0)
                .u16(fragmented ? fragmentCount : 1)
                .varint(encodedEntries.size());
        for (byte[] entry : encodedEntries) {
            putBytes(payload, entry);
        }
        return envelope(Wire.RECORD_FRAME, payload);
    }

    /** Encodes one frame entry: {@code varint signalId, u8 typeTag, value}. */
    public static byte[] encodeEntry(long signalId, Object value) {
        WireWriter w = new WireWriter(32);
        w.varint(signalId);
        writeValue(w, value);
        return w.toByteArray();
    }

    // ---- control framing and messages --------------------------------------------------------

    /** Prefixes a body with its u32 little-endian length. */
    public static byte[] frame(byte[] body) {
        if (body.length > MAX_CONTROL_BODY) {
            throw new IllegalArgumentException("control body too large: " + body.length);
        }
        byte[] out = new byte[body.length + 4];
        out[0] = (byte) body.length;
        out[1] = (byte) (body.length >>> 8);
        out[2] = (byte) (body.length >>> 16);
        out[3] = (byte) (body.length >>> 24);
        System.arraycopy(body, 0, out, 4, body.length);
        return out;
    }

    /** Length of the next framed body, or -1 when the buffer does not hold a complete length. */
    public static int framedLength(byte[] buf, int available) {
        if (available < 4) {
            return -1;
        }
        long len = (buf[0] & 0xFFL) | ((buf[1] & 0xFFL) << 8) | ((buf[2] & 0xFFL) << 16) | ((buf[3] & 0xFFL) << 24);
        if (len > MAX_CONTROL_BODY) {
            throw new IllegalStateException("control body too large: " + len);
        }
        return available >= 4 + len ? (int) len : -1;
    }

    public static byte[] encodeRequest(ClientRequest m) {
        WireWriter w = new WireWriter();
        w.u8(m.kind()).u32(m.requestId());
        switch (m.kind()) {
            case KIND_HELLO -> w.string(m.text()).u8(m.number()).u16(m.port());
            case KIND_SUBSCRIBE -> {
                w.string(m.text());
                writeRate(w, m.rate());
            }
            case KIND_UNSUBSCRIBE -> w.u32(m.id());
            case KIND_WRITE -> {
                w.varint(m.id());
                writeValue(w, m.value());
            }
            case KIND_ACTION_RUN, KIND_ACTION_CANCEL -> w.varint(m.id());
            case KIND_TIME_PING -> w.u64(m.timestamp());
            case KIND_SCHEMA_REQUEST, KIND_FILE_LIST -> {
                // no payload
            }
            case KIND_FILE_DOWNLOAD -> w.string(m.text()).u64(m.offset()).u32(m.maxLength());
            case KIND_FILE_DELETE -> w.string(m.text());
            default -> throw new IllegalArgumentException("unknown request kind " + m.kind());
        }
        return w.toByteArray();
    }

    public static ClientRequest decodeRequest(byte[] body) {
        WireReader r = new WireReader(body);
        int kind = r.u8();
        long requestId = r.u32();
        ClientRequest out = switch (kind) {
            case KIND_HELLO -> ClientRequest.hello(requestId, r.string(), r.u8(), r.u16());
            case KIND_SUBSCRIBE -> ClientRequest.subscribe(requestId, r.string(), readRate(r));
            case KIND_UNSUBSCRIBE -> ClientRequest.unsubscribe(requestId, r.u32());
            case KIND_WRITE -> {
                long signalId = r.varint();
                yield ClientRequest.write(requestId, signalId, readValue(r));
            }
            case KIND_ACTION_RUN -> ClientRequest.actionRun(requestId, r.varint());
            case KIND_ACTION_CANCEL -> ClientRequest.actionCancel(requestId, r.varint());
            case KIND_TIME_PING -> ClientRequest.timePing(requestId, r.u64());
            case KIND_SCHEMA_REQUEST -> ClientRequest.schemaRequest(requestId);
            case KIND_FILE_LIST -> ClientRequest.fileList(requestId);
            case KIND_FILE_DOWNLOAD -> ClientRequest.fileDownload(requestId, r.string(), r.u64(), (int) r.u32());
            case KIND_FILE_DELETE -> ClientRequest.fileDelete(requestId, r.string());
            default -> throw new IllegalStateException("unknown client message kind " + kind);
        };
        expectEnd(r);
        return out;
    }

    public static byte[] encodeServer(ServerMessage m) {
        WireWriter w = new WireWriter();
        if (m instanceof ServerMessage.Reply reply) {
            w.u8(KIND_REPLY).u32(reply.requestId()).u8(reply.status()).string(reply.reason());
            writeReplyData(w, reply.data());
        } else if (m instanceof ServerMessage.Schema schema) {
            w.u8(KIND_SCHEMA).u32(schema.requestId());
            writeDictionary(w, schema.dictionary());
            w.varint(schema.schemas().size());
            for (ArcpStructSchema s : schema.schemas()) {
                writeStructSchema(w, s);
            }
        } else if (m instanceof ServerMessage.TimePong pong) {
            w.u8(KIND_TIME_PONG).u32(pong.requestId()).u64(pong.clientMicros()).u64(pong.serverMicros());
        } else if (m instanceof ServerMessage.WriteReply write) {
            w.u8(KIND_WRITE_REPLY).u32(write.requestId()).u8(write.accepted() ? 1 : 0).string(write.reason());
            writeValue(w, write.actualValue());
        } else {
            throw new IllegalArgumentException("unknown server message " + m);
        }
        return w.toByteArray();
    }

    public static ServerMessage decodeServer(byte[] body) {
        WireReader r = new WireReader(body);
        int kind = r.u8();
        long requestId = r.u32();
        ServerMessage out = switch (kind) {
            case KIND_REPLY -> {
                int status = r.u8();
                String reason = r.string();
                yield new ServerMessage.Reply(requestId, status, reason, readReplyData(r));
            }
            case KIND_SCHEMA -> {
                List<ArcpDictEntry> dictionary = readDictionary(r);
                int n = count(r, 4);
                List<ArcpStructSchema> schemas = new ArrayList<>(n);
                for (int i = 0; i < n; i++) {
                    schemas.add(readStructSchema(r));
                }
                yield new ServerMessage.Schema(requestId, dictionary, schemas);
            }
            case KIND_TIME_PONG -> new ServerMessage.TimePong(requestId, r.u64(), r.u64());
            case KIND_WRITE_REPLY -> {
                boolean accepted = boolValue(r.u8());
                String reason = r.string();
                yield new ServerMessage.WriteReply(requestId, accepted, reason, readValue(r));
            }
            default -> throw new IllegalStateException("unknown server message kind " + kind);
        };
        expectEnd(r);
        return out;
    }

    private static void writeRate(WireWriter w, ArcpRate rate) {
        if (rate.onChange()) {
            w.u8(RATE_ON_CHANGE);
        } else {
            w.u8(RATE_HZ).f64(rate.hz());
        }
    }

    private static ArcpRate readRate(WireReader r) {
        return switch (r.u8()) {
            case RATE_ON_CHANGE -> ArcpRate.ON_CHANGE;
            case RATE_HZ -> ArcpRate.hz(r.f64());
            default -> throw new IllegalStateException("rate tag");
        };
    }

    private static void writeReplyData(WireWriter w, ReplyData data) {
        if (data instanceof ReplyData.Subscription sub) {
            w.u8(1).u32(sub.subscriptionId());
        } else if (data instanceof ReplyData.Session session) {
            w.u8(2).u32(session.sessionId());
        } else if (data instanceof ReplyData.Files files) {
            w.u8(3).varint(files.files().size());
            for (FileInfo f : files.files()) {
                w.string(f.name()).u64(f.size()).u64(f.modifiedUnixSecs());
            }
        } else if (data instanceof ReplyData.Chunk chunk) {
            w.u8(4).u8(chunk.eof() ? 1 : 0).varint(chunk.data().length);
            putBytes(w, chunk.data());
        } else {
            w.u8(0);
        }
    }

    private static ReplyData readReplyData(WireReader r) {
        return switch (r.u8()) {
            case 0 -> new ReplyData.None();
            case 1 -> new ReplyData.Subscription(r.u32());
            case 2 -> new ReplyData.Session(r.u32());
            case 3 -> {
                int n = count(r, 17);
                List<FileInfo> files = new ArrayList<>(n);
                for (int i = 0; i < n; i++) {
                    files.add(new FileInfo(r.string(), r.u64(), r.u64()));
                }
                yield new ReplyData.Files(files);
            }
            case 4 -> {
                boolean eof = boolValue(r.u8());
                int len = length(r);
                yield new ReplyData.Chunk(eof, r.bytes(len));
            }
            default -> throw new IllegalStateException("reply data kind");
        };
    }

    // ---- helpers -----------------------------------------------------------------------------

    /** Wraps a payload in a record envelope: u8 type, varint length, bytes. */
    public static byte[] envelope(int recordType, WireWriter payload) {
        WireWriter out = new WireWriter(payload.size() + 12);
        out.record(recordType, payload);
        return out.toByteArray();
    }

    static void putBytes(WireWriter w, byte[] bytes) {
        w.bytes(bytes, 0, bytes.length);
    }

    private static int length(WireReader r) {
        long raw = r.varint();
        if (raw < 0 || raw > r.remaining()) {
            throw new IllegalStateException("invalid length " + raw);
        }
        return (int) raw;
    }

    /** Element count; each element needs at least {@code minBytes} bytes. */
    private static int count(WireReader r, int minBytes) {
        long raw = r.varint();
        if (raw < 0 || raw * minBytes > r.remaining()) {
            throw new IllegalStateException("invalid count " + raw);
        }
        return (int) raw;
    }

    private static void expectEnd(WireReader r) {
        if (r.hasRemaining()) {
            throw new IllegalStateException("trailing bytes in control message");
        }
    }
}
