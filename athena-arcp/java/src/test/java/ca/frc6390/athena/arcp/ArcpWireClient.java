package ca.frc6390.athena.arcp;

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
import java.util.List;
import java.util.Map;

/**
 * Test-only ARCP client. It encodes the control requests it needs and decodes the replies and
 * frames the Rust server sends, using athena-runtime's reference reader. Production code never
 * encodes or decodes ARCP.
 */
final class ArcpWireClient implements AutoCloseable {

    static final int KIND_HELLO = 0x01;
    static final int KIND_SUBSCRIBE = 0x10;
    static final int KIND_WRITE = 0x20;
    static final int KIND_ACTION_RUN = 0x21;
    static final int KIND_REPLY = 0x80;
    static final int KIND_SCHEMA = 0x81;
    static final int KIND_WRITE_REPLY = 0x83;
    static final int PROTOCOL_VERSION = 2;

    /** Reply status values from the control channel. */
    static final int STATUS_OK = 0;
    static final int STATUS_REJECTED = 1;

    record Reply(int status, String reason, long subscriptionId) {
    }

    record WriteAnswer(boolean accepted, String reason, Object actualValue) {
    }

    record Frame(long timestampMicros, long sequence, Map<Long, Object> values) {
    }

    private final Socket tcp;
    private final InputStream in;
    private final OutputStream out;
    private final List<Object> pushes = new ArrayList<>();
    private int nextId = 1;

    ArcpWireClient(int controlPort) throws IOException {
        tcp = new Socket(InetAddress.getLoopbackAddress(), controlPort);
        tcp.setSoTimeout(5_000);
        in = new DataInputStream(tcp.getInputStream());
        out = tcp.getOutputStream();
        // The server pushes the dictionary as soon as the client connects.
        int kind = new WireReader(readBody()).u8();
        if (kind != KIND_SCHEMA) {
            throw new IllegalStateException("expected schema push on connect, got kind " + kind);
        }
    }

    Reply hello(int udpPort) throws IOException {
        WireWriter w = new WireWriter();
        w.string("test-client").u8(PROTOCOL_VERSION).u16(udpPort);
        return awaitReply(send(KIND_HELLO, w));
    }

    Reply subscribe(String pattern) throws IOException {
        WireWriter w = new WireWriter();
        w.string(pattern).u8(0);
        return awaitReply(send(KIND_SUBSCRIBE, w));
    }

    /** Sends a write without waiting, so the caller can service the robot side meanwhile. */
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

    WriteAnswer awaitWriteReply(int requestId) throws IOException {
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
        byte[] payloadBytes = payload.toByteArray();
        WireWriter frame = new WireWriter();
        frame.u32(1 + 4 + payloadBytes.length).u8(kind).u32(id).bytes(payloadBytes, 0, payloadBytes.length);
        out.write(frame.toByteArray());
        out.flush();
        return id;
    }

    /** Reads messages until the one with {@code requestId} arrives. Other messages are kept as pushes. */
    private Object awaitMessage(int requestId) throws IOException {
        while (true) {
            byte[] body = readBody();
            WireReader r = new WireReader(body);
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

    private static Object decode(int kind, WireReader r) {
        switch (kind) {
            case KIND_REPLY: {
                int status = r.u8();
                String reason = r.string();
                // Reply data tag 1 is a subscription id. Other data is not used by these tests.
                long subscriptionId = r.u8() == 1 ? r.u32() : -1;
                return new Reply(status, reason, subscriptionId);
            }
            case KIND_WRITE_REPLY: {
                boolean accepted = r.u8() != 0;
                String reason = r.string();
                return new WriteAnswer(accepted, reason, readValue(r));
            }
            case KIND_SCHEMA:
                return "schema";
            default:
                throw new IllegalStateException("unexpected server message kind " + kind);
        }
    }

    /** Frames received on the UDP socket, one entry per datagram record. Other records are skipped. */
    static List<Frame> receiveFrames(DatagramSocket udp, long budgetMillis) throws IOException {
        List<Frame> frames = new ArrayList<>();
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
                if (record.type() == Wire.RECORD_FRAME) {
                    frames.add(decodeFrame(record.payload()));
                }
            }
        }
        return frames;
    }

    private static Frame decodeFrame(WireReader p) {
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

    /** Decodes a typed value (tag byte then body). Covers the scalar and string types used here. */
    static Object readValue(WireReader r) {
        int tag = r.u8();
        switch (tag) {
            case 1:
                return r.u8() != 0;
            case 2:
                return r.zigzag();
            case 3:
                return r.f64();
            case 4:
                return r.string();
            default:
                throw new IllegalStateException("value tag " + tag + " not supported by the test client");
        }
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
