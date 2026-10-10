package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Test-only ARCP client for the robot bridge. It encodes control requests and decodes replies, the dictionary,
 * realtime frames, markers and replay records with athena-runtime's reference reader. Production code never
 * encodes or decodes ARCP.
 */
final class ArcpTestClient implements AutoCloseable {
    static final int KIND_HELLO = 0x01;
    static final int KIND_SUBSCRIBE = 0x10;
    static final int KIND_REPLAY_SUBSCRIBE = 0x12;
    static final int KIND_WRITE = 0x20;
    static final int KIND_ACTION_RUN = 0x21;
    static final int KIND_ACTION_CANCEL = 0x22;
    static final int KIND_FILE_LIST = 0x50;
    static final int KIND_FILE_DOWNLOAD = 0x51;
    static final int KIND_FILE_DELETE = 0x52;
    static final int KIND_BACKFILL = 0x60;
    static final int KIND_REPLY = 0x80;
    static final int KIND_SCHEMA = 0x81;
    static final int KIND_WRITE_REPLY = 0x83;
    static final int STATUS_OK = 0;

    /** A control reply: status, reason and the data of the reply. */
    record Reply(int status, String reason, Object data) {
    }

    record WriteAnswer(boolean accepted, String reason, Object actualValue) {
    }

    record Dict(long id, String path, int typeTag, int role, int flags, String lockGroup) {
    }

    /** A realtime frame: timestamp, sequence and the changed values by signal id. */
    record Frame(long timestampMicros, long sequence, Map<Long, Object> values) {
    }

    /** A marker record. */
    record Marker(long timestampMicros, int kind, String message, long signalId) {
    }

    private final Socket tcp;
    private final InputStream in;
    private final OutputStream out;
    private final List<Object> pushes = new ArrayList<>();
    private final Map<String, Dict> dictionary = new LinkedHashMap<>();
    private int nextId = 1;

    ArcpTestClient(int controlPort) throws IOException {
        tcp = new Socket(InetAddress.getLoopbackAddress(), controlPort);
        tcp.setSoTimeout(5_000);
        in = new DataInputStream(tcp.getInputStream());
        out = tcp.getOutputStream();
        WireReader schema = new WireReader(readBody());
        if (schema.u8() != KIND_SCHEMA) {
            throw new IllegalStateException("expected the schema on connect");
        }
        schema.u32();
        readDictionary(schema);
    }

    /** Dictionary entries by path, from the schema push on connect or the latest push. */
    Map<String, Dict> dictionary() {
        return dictionary;
    }

    Dict signal(String path) {
        Dict dict = dictionary.get(path);
        if (dict == null) {
            throw new IllegalStateException("no signal " + path + " in " + dictionary.keySet());
        }
        return dict;
    }

    Reply hello(int udpPort) throws IOException {
        WireWriter w = new WireWriter();
        w.string("test-client").u8(2).u16(udpPort);
        return awaitReply(send(KIND_HELLO, w));
    }

    Reply subscribe(String pattern) throws IOException {
        WireWriter w = new WireWriter();
        w.string(pattern).u8(0);
        return awaitReply(send(KIND_SUBSCRIBE, w));
    }

    Reply replaySubscribe() throws IOException {
        return awaitReply(send(KIND_REPLAY_SUBSCRIBE, new WireWriter()));
    }

    Reply backfill(long from, long to) throws IOException {
        WireWriter w = new WireWriter();
        w.u64(from).u64(to);
        return awaitReply(send(KIND_BACKFILL, w));
    }

    Reply fileList() throws IOException {
        return awaitReply(send(KIND_FILE_LIST, new WireWriter()));
    }

    Reply fileDelete(String name) throws IOException {
        WireWriter w = new WireWriter();
        w.string(name);
        return awaitReply(send(KIND_FILE_DELETE, w));
    }

    Reply fileDownload(String name) throws IOException {
        WireWriter w = new WireWriter();
        w.string(name).u64(0).u32(60_000);
        return awaitReply(send(KIND_FILE_DOWNLOAD, w));
    }

    /** Sends a write without waiting; the caller services the robot side and then calls {@link #awaitWrite}. */
    int sendWrite(long signalId, Object value) throws IOException {
        WireWriter w = new WireWriter();
        w.varint(signalId);
        writeValue(w, value);
        return send(KIND_WRITE, w);
    }

    int sendAction(long signalId) throws IOException {
        WireWriter w = new WireWriter();
        w.varint(signalId);
        return send(KIND_ACTION_RUN, w);
    }

    int sendActionCancel(long signalId) throws IOException {
        WireWriter w = new WireWriter();
        w.varint(signalId);
        return send(KIND_ACTION_CANCEL, w);
    }

    WriteAnswer awaitWrite(int requestId) throws IOException {
        Object message = awaitMessage(requestId);
        if (!(message instanceof WriteAnswer answer)) {
            throw new IllegalStateException("expected write reply, got " + message);
        }
        return answer;
    }

    Reply awaitReply(int requestId) throws IOException {
        Object message = awaitMessage(requestId);
        if (!(message instanceof Reply reply)) {
            throw new IllegalStateException("expected reply, got " + message);
        }
        return reply;
    }

    private int send(int kind, WireWriter payload) throws IOException {
        int id = nextId++;
        byte[] body = payload.toByteArray();
        WireWriter frame = new WireWriter();
        frame.u32(1 + 4 + body.length).u8(kind).u32(id).bytes(body, 0, body.length);
        out.write(frame.toByteArray());
        out.flush();
        return id;
    }

    private Object awaitMessage(int requestId) throws IOException {
        while (true) {
            WireReader r = new WireReader(readBody());
            int kind = r.u8();
            long id = r.u32();
            Object message = decode(kind, r);
            if (id == requestId) {
                return message;
            }
            pushes.add(message);
        }
    }

    private byte[] readBody() throws IOException {
        byte[] header = in.readNBytes(4);
        if (header.length < 4) {
            throw new IOException("control connection closed");
        }
        int len = (header[0] & 0xFF) | ((header[1] & 0xFF) << 8) | ((header[2] & 0xFF) << 16) | ((header[3] & 0xFF) << 24);
        return in.readNBytes(len);
    }

    private Object decode(int kind, WireReader r) {
        switch (kind) {
            case KIND_REPLY -> {
                int status = r.u8();
                String reason = r.string();
                return new Reply(status, reason, readReplyData(r));
            }
            case KIND_WRITE_REPLY -> {
                boolean accepted = r.u8() != 0;
                String reason = r.string();
                return new WriteAnswer(accepted, reason, readValue(r));
            }
            case KIND_SCHEMA -> {
                readDictionary(r);
                return "schema";
            }
            default -> throw new IllegalStateException("unexpected server message kind " + kind);
        }
    }

    private Object readReplyData(WireReader r) {
        int tag = r.u8();
        switch (tag) {
            case 0:
                return null;
            case 1:
                return r.u32();
            case 2:
                return r.u32();
            case 3: {
                long n = r.varint();
                List<String> names = new ArrayList<>();
                for (long i = 0; i < n; i++) {
                    names.add(r.string());
                    r.u64();
                    r.u64();
                }
                return names;
            }
            case 4: {
                boolean eof = r.u8() != 0;
                int len = (int) r.varint();
                return new Object[] {eof, r.bytes(len)};
            }
            case 5: {
                int len = (int) r.varint();
                return r.bytes(len);
            }
            default:
                throw new IllegalStateException("reply data tag " + tag);
        }
    }

    private void readDictionary(WireReader r) {
        dictionary.clear();
        long n = r.varint();
        for (long i = 0; i < n; i++) {
            long id = r.varint();
            String path = r.string();
            int typeTag = r.u8();
            r.string();
            int role = r.u8();
            int flags = r.u8();
            String lockGroup = r.string();
            r.varint();
            dictionary.put(path, new Dict(id, path, typeTag, role, flags, lockGroup));
        }
    }

    /** Frames received on a UDP socket, one entry per frame record. */
    static List<Frame> receiveFrames(DatagramSocket udp, long budgetMillis) throws IOException {
        List<Frame> frames = new ArrayList<>();
        for (WireRecord record : receiveRecords(udp, budgetMillis)) {
            if (record.type() == Wire.RECORD_FRAME) {
                frames.add(decodeFrame(record.payload()));
            }
        }
        return frames;
    }

    /** Markers received on a UDP socket. */
    static List<Marker> receiveMarkers(DatagramSocket udp, long budgetMillis) throws IOException {
        List<Marker> markers = new ArrayList<>();
        for (WireRecord record : receiveRecords(udp, budgetMillis)) {
            if (record.type() == 0x03) {
                WireReader p = record.payload();
                markers.add(new Marker(p.zigzag(), (int) p.varint(), p.string(), p.varint()));
            }
        }
        return markers;
    }

    /** Every record received on a UDP socket within the budget, in arrival order. */
    static List<WireRecord> receiveRecords(DatagramSocket udp, long budgetMillis) throws IOException {
        List<WireRecord> records = new ArrayList<>();
        long deadline = System.currentTimeMillis() + budgetMillis;
        byte[] buf = new byte[65_536];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        while (System.currentTimeMillis() < deadline) {
            try {
                udp.setSoTimeout((int) Math.max(1, deadline - System.currentTimeMillis()));
                udp.receive(packet);
            } catch (SocketTimeoutException timeout) {
                break;
            }
            WireReader datagram = new WireReader(Arrays.copyOf(packet.getData(), packet.getLength()));
            while (datagram.hasRemaining()) {
                WireReader.Record record = datagram.record();
                records.add(new WireRecord(record.type(), record.payload()));
            }
        }
        return records;
    }

    /** A received record: type and payload bytes. */
    record WireRecord(int type, WireReader payload) {
    }

    /** Decodes a frame payload. */
    static Frame decodeFrame(WireReader p) {
        p.u8();
        p.u8();
        long timestamp = p.u64();
        long sequence = p.u32();
        p.u16();
        p.u16();
        long n = p.varint();
        Map<Long, Object> values = new HashMap<>();
        for (long i = 0; i < n; i++) {
            long id = p.varint();
            values.put(id, readValue(p));
        }
        return new Frame(timestamp, sequence, values);
    }

    static Object readValue(WireReader r) {
        int tag = r.u8();
        return switch (tag) {
            case 1 -> r.u8() != 0;
            case 2 -> r.zigzag();
            case 3 -> r.f64();
            case 4 -> r.string();
            default -> throw new IllegalStateException("value tag " + tag + " not supported by the test client");
        };
    }

    private static void writeValue(WireWriter w, Object value) {
        if (value instanceof Boolean b) {
            w.u8(1).u8(b ? 1 : 0);
        } else if (value instanceof Long l) {
            w.u8(2).zigzag(l);
        } else if (value instanceof Double d) {
            w.u8(3).f64(d);
        } else if (value instanceof String s) {
            w.u8(4).string(s);
        } else {
            throw new IllegalArgumentException("test client cannot encode " + value);
        }
    }

    @Override
    public void close() throws IOException {
        tcp.close();
    }
}
