package ca.frc6390.athena.arcp.v2;

import ca.frc6390.athena.arcp.v2.ArcpMessages.ClientRequest;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ServerMessage;
import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.LongFunction;
import java.net.SocketTimeoutException;

/** Test-only control client and UDP frame reader. */
final class ArcpTestClient implements AutoCloseable {

    /** A decoded frame datagram. */
    record Frame(boolean keyframe, boolean fragmented, long timestamp, long sequence, int index, int count,
            List<ArcpCycle.Entry> entries) {
    }

    private final Socket tcp;
    private final InputStream in;
    private final OutputStream out;
    private final List<ServerMessage> pushes = new ArrayList<>();
    private long nextId = 1;

    ArcpTestClient(int port) throws IOException {
        tcp = new Socket(InetAddress.getLoopbackAddress(), port);
        tcp.setSoTimeout(5000);
        in = tcp.getInputStream();
        out = tcp.getOutputStream();
        ServerMessage first = readMessage();
        if (!(first instanceof ServerMessage.Schema)) {
            throw new IllegalStateException("expected schema on connect, got " + first);
        }
    }

    ServerMessage readMessage() throws IOException {
        byte[] header = in.readNBytes(4);
        int len = (header[0] & 0xFF) | ((header[1] & 0xFF) << 8) | ((header[2] & 0xFF) << 16) | ((header[3] & 0xFF) << 24);
        return ArcpCodec.decodeServer(in.readNBytes(len));
    }

    /** Sends a request built from the next id and returns the matching server message. */
    ServerMessage request(LongFunction<ClientRequest> factory) throws IOException {
        long id = nextId++;
        out.write(ArcpCodec.frame(ArcpCodec.encodeRequest(factory.apply(id))));
        out.flush();
        while (true) {
            ServerMessage m = readMessage();
            long rid;
            if (m instanceof ServerMessage.Reply r) {
                rid = r.requestId();
            } else if (m instanceof ServerMessage.WriteReply wr) {
                rid = wr.requestId();
            } else if (m instanceof ServerMessage.Schema sc) {
                rid = sc.requestId();
            } else {
                rid = ((ServerMessage.TimePong) m).requestId();
            }
            if (rid == id) {
                return m;
            }
            pushes.add(m);
        }
    }

    /** Sends a write and returns the robot's {accepted, reason, actualValue} reply. */
    ServerMessage.WriteReply write(long signalId, Object value) throws IOException {
        ServerMessage m = request(id -> ClientRequest.write(id, signalId, value));
        if (!(m instanceof ServerMessage.WriteReply wr)) {
            throw new IllegalStateException("expected write reply, got " + m);
        }
        return wr;
    }

    ArcpMessages.ServerMessage.Reply reply(LongFunction<ClientRequest> factory) throws IOException {
        return (ServerMessage.Reply) request(factory);
    }

    void hello(int udpPort) throws IOException {
        ServerMessage.Reply r = reply(id -> ClientRequest.hello(id, "test", ArcpCodec.PROTOCOL_VERSION, udpPort));
        if (r.status() != ArcpMessages.STATUS_OK) {
            throw new IllegalStateException(r.reason());
        }
    }

    long subscribe(String pattern, ArcpRate rate) throws IOException {
        ServerMessage.Reply r = reply(id -> ClientRequest.subscribe(id, pattern, rate));
        if (r.status() != ArcpMessages.STATUS_OK) {
            throw new IllegalStateException(r.reason());
        }
        return ((ArcpMessages.ReplyData.Subscription) r.data()).subscriptionId();
    }

    List<ServerMessage> pushes() {
        return pushes;
    }

    @Override
    public void close() throws IOException {
        tcp.close();
    }

    /** Receives datagrams until the socket times out. Non-frame records are skipped. */
    static List<Frame> drainFrames(DatagramSocket udp) throws IOException {
        List<Frame> out = new ArrayList<>();
        byte[] buf = new byte[65536];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        while (true) {
            try {
                udp.receive(packet);
            } catch (SocketTimeoutException timeout) {
                return out;
            }
            out.addAll(parseDatagram(packet.getData(), packet.getLength()));
        }
    }

    /** Parses one datagram into its frame records. */
    static List<Frame> parseDatagram(byte[] data, int length) {
        List<Frame> out = new ArrayList<>();
        WireReader r = new WireReader(Arrays.copyOf(data, length));
        while (r.hasRemaining()) {
            WireReader.Record record = r.record();
            if (record.type() == Wire.RECORD_FRAME) {
                out.add(decodeFrame(record.payload()));
            }
        }
        return out;
    }

    static List<ArcpCycle.Entry> drainEntries(DatagramSocket udp) throws IOException {
        List<ArcpCycle.Entry> entries = new ArrayList<>();
        for (Frame f : drainFrames(udp)) {
            entries.addAll(f.entries());
        }
        return entries;
    }

    private static Frame decodeFrame(WireReader p) {
        p.u8();
        int flags = p.u8();
        long ts = p.u64();
        long seq = p.u32();
        int index = p.u16();
        int count = p.u16();
        long n = p.varint();
        List<ArcpCycle.Entry> entries = new ArrayList<>();
        for (long i = 0; i < n; i++) {
            long id = p.varint();
            entries.add(new ArcpCycle.Entry(id, ArcpCodec.readValue(p)));
        }
        return new Frame((flags & Wire.FRAME_FLAG_KEYFRAME) != 0, (flags & Wire.FRAME_FLAG_FRAGMENT) != 0,
                ts, seq, index, count, entries);
    }
}
