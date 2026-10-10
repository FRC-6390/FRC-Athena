package ca.frc6390.athena.arcp.v2;

import ca.frc6390.athena.arcp.v2.ArcpMessages.ClientRequest;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ReplyData;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ServerMessage;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Pure-Java ARCP v2 server (same wire behaviour as the Rust {@code arcp_server::v2}).
 *
 * <ul>
 *   <li>Realtime (E1, E2): {@link #publishCycle} sends one batched frame per subscriber per cycle over
 *       UDP, containing only subscribed signals that are due (on change, or at the requested Hz).
 *       Keyframes are sent at least once per {@link Config#keyframeIntervalMicros()}.</li>
 *   <li>Schema (E3): the dictionary is pushed on connect and on change.</li>
 *   <li>Control (E4, E5): TCP with typed requests and structured replies. Writes and actions are
 *       forwarded to the {@link WriteHandler}; the server never pre-validates them.</li>
 *   <li>Files (E6): list, download and delete through {@link ArcpFileService}.</li>
 * </ul>
 *
 * <p>Threads are daemon threads. Methods of this class are thread safe.</p>
 */
public final class ArcpServer implements ArcpSink, AutoCloseable {

    /** Server configuration. */
    public record Config(
            int controlPort,
            Path fileRoot,
            long budgetBytesPerSecond,
            int mtu,
            long keyframeIntervalMicros) {

        /** Ephemeral control port, no file service, no budget, 1200-byte datagrams, 1 s keyframes. */
        public static Config defaults() {
            return new Config(0, null, 0L, 1200, 1_000_000L);
        }
    }

    /** A subscription whose effective rate was lowered by the bandwidth budget. */
    public record Throttled(int sessionId, int subscriptionId, String pattern, ArcpRate requested, int divisor) {
    }

    /** Server status snapshot. */
    public record Status(int sessions, int subscriptions, List<Throttled> throttled, long bytesLastWindow) {
    }

    private static final int FRAME_OVERHEAD = 64;
    private static final int MAX_DIVISOR = 16;
    private static final long WINDOW_MICROS = 1_000_000L;

    private final Config config;
    private final WriteHandler handler;
    private final ArcpFileService files;
    private final long startNanos = System.nanoTime();
    private final Object lock = new Object();
    private final ServerSocket listener;
    private final DatagramSocket udp;
    private final List<Session> sessions = new ArrayList<>();
    private List<ArcpDictEntry> dictionary = List.of();
    private List<ArcpStructSchema> schemas = List.of();
    private final Map<Long, Object> latest = new HashMap<>();
    private long latestTs;
    private int nextSessionId = 1;
    private long windowStart;
    private long windowBytes;
    private long lastWindowBytes;
    private volatile Nt4Bridge nt4 = Nt4Bridge.DISABLED;
    private volatile boolean closed;

    /** Subscription state for one session. */
    private static final class Subscription {
        final int id;
        final String pattern;
        final ArcpRate rate;
        int divisor = 1;
        long[] matches = new long[0];

        Subscription(int id, String pattern, ArcpRate rate) {
            this.id = id;
            this.pattern = pattern;
            this.rate = rate;
        }

        /** Negative means on change; otherwise the effective Hz. */
        double effectiveHz() {
            return rate.onChange() ? -1.0 : rate.hz() / divisor;
        }
    }

    /** Last value sent to one subscriber for one signal. */
    private record Sent(long at, Object value) {
    }

    /** One connected control client. */
    private static final class Session {
        final int id;
        final Socket socket;
        final OutputStream out;
        final Object writeLock = new Object();
        final List<Subscription> subs = new ArrayList<>();
        final Map<Long, Sent> sent = new HashMap<>();
        InetSocketAddress udpTarget;
        boolean helloDone;
        int nextSubId = 1;
        long seq;
        Long lastKeyframe;
        volatile boolean dead;

        Session(int id, Socket socket, OutputStream out) {
            this.id = id;
            this.socket = socket;
            this.out = out;
        }
    }

    /**
     * Binds the control listener and the UDP sender, then starts accepting connections.
     *
     * @param config server configuration
     * @param handler robot-side write and action decisions
     * @throws IOException when the sockets cannot be bound
     */
    public ArcpServer(Config config, WriteHandler handler) throws IOException {
        this.config = config;
        this.handler = handler;
        this.files = config.fileRoot() == null ? null : new ArcpFileService(config.fileRoot());
        this.listener = new ServerSocket(config.controlPort());
        this.udp = new DatagramSocket(0);
        Thread accept = new Thread(this::acceptLoop, "arcp-v2-accept");
        accept.setDaemon(true);
        accept.start();
    }

    /** @return the bound TCP control port */
    public int controlPort() {
        return listener.getLocalPort();
    }

    /** Installs an NT4 bridge. Call before publishing for the bridge to see every value. */
    public void setNt4Bridge(Nt4Bridge bridge) {
        this.nt4 = bridge == null ? Nt4Bridge.DISABLED : bridge;
    }

    @Override
    public void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> structSchemas) {
        synchronized (lock) {
            dictionary = List.copyOf(entries);
            schemas = List.copyOf(structSchemas);
            for (Session s : sessions) {
                for (Subscription sub : s.subs) {
                    sub.matches = matchSignals(sub.pattern);
                }
            }
            pushSchema();
            sessions.removeIf(s -> s.dead);
        }
        nt4.dictionaryChanged(dictionary);
    }

    @Override
    public void publishCycle(long cycleTimestampMicros, ArcpCycle cycle) {
        List<ArcpDictEntry> byId;
        List<Long> changed = new ArrayList<>();
        synchronized (lock) {
            for (ArcpCycle.Entry e : cycle.entries()) {
                Object old = latest.get(e.signalId());
                if (old == null || !ArcpCodec.sameValue(old, e.value())) {
                    latest.put(e.signalId(), e.value());
                    changed.add(e.signalId());
                }
            }
            latestTs = cycleTimestampMicros;
            rollWindow(cycleTimestampMicros);
            long bytes = 0;
            for (Session s : sessions) {
                bytes += sendDue(s, cycleTimestampMicros);
            }
            windowBytes += bytes;
            sessions.removeIf(s -> s.dead);
            byId = dictionary;
        }
        if (!changed.isEmpty() && nt4 != Nt4Bridge.DISABLED) {
            for (Long id : changed) {
                for (ArcpDictEntry entry : byId) {
                    if (entry.signalId() == id) {
                        nt4.valueChanged(entry, latest.get(id));
                    }
                }
            }
        }
    }

    @Override
    public void marker(long timestampMicros, int kind, String message, long signalId) {
        byte[] record = ArcpCodec.markerRecord(timestampMicros, kind, message == null ? "" : message, signalId);
        synchronized (lock) {
            for (Session s : sessions) {
                sendDatagram(s, record);
            }
        }
    }

    /** @return a snapshot of connections, subscriptions and budget throttling */
    public Status status() {
        synchronized (lock) {
            List<Throttled> throttled = new ArrayList<>();
            int subscriptions = 0;
            for (Session s : sessions) {
                for (Subscription sub : s.subs) {
                    subscriptions++;
                    if (sub.divisor > 1) {
                        throttled.add(new Throttled(s.id, sub.id, sub.pattern, sub.rate, sub.divisor));
                    }
                }
            }
            return new Status(sessions.size(), subscriptions, throttled, lastWindowBytes);
        }
    }

    /** Stops accepting connections and closes every client socket. */
    @Override
    public void close() {
        closed = true;
        try {
            listener.close();
        } catch (IOException ignored) {
            // already closing
        }
        synchronized (lock) {
            for (Session s : sessions) {
                closeQuietly(s.socket);
            }
            sessions.clear();
        }
        udp.close();
    }

    // ---- accept and control ------------------------------------------------------------------

    private void acceptLoop() {
        while (!closed) {
            try {
                Socket socket = listener.accept();
                Thread t = new Thread(() -> serve(socket), "arcp-v2-conn");
                t.setDaemon(true);
                t.start();
            } catch (IOException e) {
                if (closed) {
                    return;
                }
            }
        }
    }

    private void serve(Socket socket) {
        Session session;
        try {
            socket.setTcpNoDelay(true);
            OutputStream out = socket.getOutputStream();
            synchronized (lock) {
                session = new Session(nextSessionId++, socket, out);
                sessions.add(session);
                if (!writeMessage(session, new ServerMessage.Schema(0, dictionary, schemas))) {
                    sessions.remove(session);
                    closeQuietly(socket);
                    return;
                }
            }
        } catch (IOException e) {
            closeQuietly(socket);
            return;
        }
        try {
            InputStream in = socket.getInputStream();
            byte[] acc = new byte[4096];
            int len = 0;
            while (true) {
                if (len == acc.length) {
                    acc = Arrays.copyOf(acc, acc.length * 2);
                }
                int n = in.read(acc, len, acc.length - len);
                if (n < 0) {
                    break;
                }
                len += n;
                int start = 0;
                boolean keepOpen = true;
                while (true) {
                    int bodyLen = frameLength(acc, start, len - start);
                    if (bodyLen < 0) {
                        break;
                    }
                    byte[] body = Arrays.copyOfRange(acc, start + 4, start + 4 + bodyLen);
                    start += 4 + bodyLen;
                    ClientRequest request;
                    try {
                        request = ArcpCodec.decodeRequest(body);
                    } catch (RuntimeException e) {
                        writeMessage(session, reply(0, ArcpMessages.STATUS_ERROR, "malformed request", new ReplyData.None()));
                        keepOpen = false;
                        break;
                    }
                    if (!handle(session, socket, request)) {
                        keepOpen = false;
                        break;
                    }
                }
                if (!keepOpen) {
                    break;
                }
                System.arraycopy(acc, start, acc, 0, len - start);
                len -= start;
            }
        } catch (IOException ignored) {
            // client went away
        } finally {
            synchronized (lock) {
                sessions.remove(session);
            }
            closeQuietly(socket);
        }
    }

    /** Handles one request. Returns false when the connection should close. */
    private boolean handle(Session session, Socket socket, ClientRequest req) {
        int kind = req.kind();
        if (kind == ArcpCodec.KIND_TIME_PING) {
            return writeMessage(session, new ServerMessage.TimePong(req.requestId(), req.timestamp(), elapsedMicros()));
        }
        if (kind == ArcpCodec.KIND_SCHEMA_REQUEST) {
            synchronized (lock) {
                return writeMessage(session, new ServerMessage.Schema(req.requestId(), dictionary, schemas));
            }
        }
        if (kind == ArcpCodec.KIND_HELLO) {
            if (req.number() != ArcpCodec.PROTOCOL_VERSION) {
                return replyError(session, req, "unsupported protocol version");
            }
            synchronized (lock) {
                session.udpTarget = new InetSocketAddress(socket.getInetAddress(), req.port());
                session.helloDone = true;
            }
            return writeMessage(session, reply(req.requestId(), ArcpMessages.STATUS_OK, "",
                    new ReplyData.Session(session.id)));
        }
        if (!session.helloDone) {
            return replyError(session, req, "hello required");
        }
        switch (kind) {
            case ArcpCodec.KIND_SUBSCRIBE:
                return subscribe(session, req);
            case ArcpCodec.KIND_UNSUBSCRIBE:
                return unsubscribe(session, req);
            case ArcpCodec.KIND_WRITE:
                return writeRequest(session, req);
            case ArcpCodec.KIND_ACTION_RUN:
                return robotReply(session, req.requestId(), () -> handler.action(req.id(), true));
            case ArcpCodec.KIND_ACTION_CANCEL:
                return robotReply(session, req.requestId(), () -> handler.action(req.id(), false));
            case ArcpCodec.KIND_FILE_LIST:
                return fileRequest(session, req, () -> new ReplyData.Files(requireFiles().list()));
            case ArcpCodec.KIND_FILE_DOWNLOAD:
                return fileRequest(session, req,
                        () -> requireFiles().download(req.text(), req.offset(), req.maxLength()));
            case ArcpCodec.KIND_FILE_DELETE:
                return fileRequest(session, req, () -> {
                    requireFiles().delete(req.text());
                    return new ReplyData.None();
                });
            default:
                return replyError(session, req, "unknown request");
        }
    }

    private boolean subscribe(Session session, ClientRequest req) {
        String error = ArcpPattern.validationError(req.text());
        if (error == null) {
            error = req.rate() == null ? "missing rate" : req.rate().validationError();
        }
        if (error != null) {
            return replyError(session, req, error);
        }
        synchronized (lock) {
            Subscription sub = new Subscription(session.nextSubId++, req.text(), req.rate());
            sub.matches = matchSignals(req.text());
            session.subs.add(sub);
            snapshot(session, sub);
            return writeMessage(session, reply(req.requestId(), ArcpMessages.STATUS_OK, "",
                    new ReplyData.Subscription(sub.id)));
        }
    }

    private boolean unsubscribe(Session session, ClientRequest req) {
        boolean removed;
        synchronized (lock) {
            removed = session.subs.removeIf(sub -> sub.id == req.id());
        }
        if (!removed) {
            return replyError(session, req, "no such subscription");
        }
        return writeMessage(session, reply(req.requestId(), ArcpMessages.STATUS_OK, "", new ReplyData.None()));
    }

    /**
     * Forwards a write to the robot without pre-checking it, and replies with the robot's
     * accept or reject, reason and actual value.
     */
    private boolean writeRequest(Session session, ClientRequest req) {
        WriteResult result;
        try {
            result = handler.write(req.id(), req.value());
        } catch (RuntimeException e) {
            result = WriteResult.reject(String.valueOf(e.getMessage()), req.value());
        }
        return writeMessage(session, new ServerMessage.WriteReply(req.requestId(), result.accepted(),
                result.reason(), result.actualValue()));
    }

    /** Runs a robot decision outside the state lock and replies with its outcome. */
    private boolean robotReply(Session session, long requestId, RobotCall call) {
        WriteResult result;
        try {
            result = call.run();
        } catch (RuntimeException e) {
            result = WriteResult.reject(String.valueOf(e.getMessage()), Boolean.FALSE);
        }
        if (result.accepted()) {
            return writeMessage(session, reply(requestId, ArcpMessages.STATUS_OK, "", new ReplyData.None()));
        }
        return writeMessage(session, reply(requestId, ArcpMessages.STATUS_REJECTED, result.reason(),
                new ReplyData.None()));
    }

    private boolean fileRequest(Session session, ClientRequest req, FileCall call) {
        try {
            ReplyData data = call.run();
            return writeMessage(session, reply(req.requestId(), ArcpMessages.STATUS_OK, "", data));
        } catch (IllegalArgumentException | IOException e) {
            return replyError(session, req, e.getMessage() == null ? "file request failed" : e.getMessage());
        }
    }

    private ArcpFileService requireFiles() {
        if (files == null) {
            throw new IllegalArgumentException("file service not configured");
        }
        return files;
    }

    private boolean replyError(Session session, ClientRequest req, String reason) {
        return writeMessage(session, reply(req.requestId(), ArcpMessages.STATUS_ERROR, reason, new ReplyData.None()));
    }

    private static ServerMessage reply(long requestId, int status, String reason, ReplyData data) {
        return new ServerMessage.Reply(requestId, status, reason, data);
    }

    private interface RobotCall {
        WriteResult run();
    }

    private interface FileCall {
        ReplyData run() throws IOException;
    }

    // ---- realtime ----------------------------------------------------------------------------

    /** Pushes the schema to every session. Caller holds {@link #lock}. */
    private void pushSchema() {
        ServerMessage schema = new ServerMessage.Schema(0, dictionary, schemas);
        for (Session s : sessions) {
            if (!writeMessage(s, schema)) {
                s.dead = true;
            }
        }
    }

    private long[] matchSignals(String pattern) {
        Set<Long> ids = new HashSet<>();
        for (ArcpDictEntry e : dictionary) {
            if (ArcpPattern.matches(pattern, e.path())) {
                ids.add(e.signalId());
            }
        }
        return ids.stream().mapToLong(Long::longValue).sorted().toArray();
    }

    /** Sends the latest values of signals newly matched by {@code sub}. Caller holds the lock. */
    private void snapshot(Session session, Subscription sub) {
        List<ArcpCycle.Entry> entries = new ArrayList<>();
        for (long id : sub.matches) {
            if (session.sent.containsKey(id)) {
                continue;
            }
            Object value = latest.get(id);
            if (value != null) {
                entries.add(new ArcpCycle.Entry(id, value));
            }
        }
        if (entries.isEmpty()) {
            return;
        }
        for (ArcpCycle.Entry e : entries) {
            session.sent.put(e.signalId(), new Sent(latestTs, e.value()));
        }
        if (session.lastKeyframe == null) {
            session.lastKeyframe = latestTs;
        }
        long seq = nextSeq(session);
        windowBytes += sendFrames(session, false, latestTs, seq, entries);
    }

    /** Decides which subscribed signals are due and sends one batched frame. Returns bytes sent. */
    private long sendDue(Session session, long ts) {
        if (session.udpTarget == null || session.subs.isEmpty()) {
            return 0;
        }
        TreeMap<Long, Double> policy = new TreeMap<>();
        for (Subscription sub : session.subs) {
            double hz = sub.effectiveHz();
            for (long id : sub.matches) {
                policy.merge(id, hz, (a, b) -> (a < 0 && b < 0) ? -1.0 : Math.max(a, b));
            }
        }
        boolean dueKeyframe = session.lastKeyframe == null
                || ts - session.lastKeyframe >= config.keyframeIntervalMicros();
        List<ArcpCycle.Entry> entries = new ArrayList<>();
        for (Map.Entry<Long, Double> p : policy.entrySet()) {
            long id = p.getKey();
            Object value = latest.get(id);
            if (value == null) {
                continue;
            }
            Sent prev = session.sent.get(id);
            boolean send;
            if (prev == null || dueKeyframe) {
                send = true;
            } else if (p.getValue() < 0) {
                send = !ArcpCodec.sameValue(prev.value(), value);
            } else {
                send = (double) (ts - prev.at()) >= 0.9 * 1_000_000.0 / p.getValue();
            }
            if (send) {
                session.sent.put(id, new Sent(ts, value));
                entries.add(new ArcpCycle.Entry(id, value));
            }
        }
        if (entries.isEmpty()) {
            return 0;
        }
        if (dueKeyframe) {
            session.lastKeyframe = ts;
        }
        return sendFrames(session, dueKeyframe, ts, nextSeq(session), entries);
    }

    /** Splits entries into datagram-sized fragments and sends them. Returns bytes sent. */
    private long sendFrames(Session session, boolean keyframe, long ts, long seq, List<ArcpCycle.Entry> entries) {
        List<byte[]> encoded = new ArrayList<>(entries.size());
        for (ArcpCycle.Entry e : entries) {
            encoded.add(ArcpCodec.encodeEntry(e.signalId(), e.value()));
        }
        int budget = Math.max(1, config.mtu() - FRAME_OVERHEAD);
        List<List<byte[]>> chunks = new ArrayList<>();
        chunks.add(new ArrayList<>());
        int used = 0;
        for (byte[] bytes : encoded) {
            if (used + bytes.length > budget && !chunks.get(chunks.size() - 1).isEmpty()) {
                chunks.add(new ArrayList<>());
                used = 0;
            }
            chunks.get(chunks.size() - 1).add(bytes);
            used += bytes.length;
        }
        boolean fragmented = chunks.size() > 1;
        long sent = 0;
        for (int i = 0; i < chunks.size(); i++) {
            byte[] record = ArcpCodec.frameRecord(keyframe, fragmented, ts, seq, i, chunks.size(), chunks.get(i));
            sent += sendDatagram(session, record) ? record.length : 0;
        }
        return sent;
    }

    private boolean sendDatagram(Session session, byte[] record) {
        InetSocketAddress target = session.udpTarget;
        if (target == null) {
            return false;
        }
        try {
            udp.send(new DatagramPacket(record, record.length, target));
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    private void rollWindow(long ts) {
        if (ts - windowStart < WINDOW_MICROS) {
            return;
        }
        long used = windowBytes;
        lastWindowBytes = used;
        long budget = config.budgetBytesPerSecond();
        if (budget > 0) {
            if (used > budget) {
                // Lowest priority = newest session and newest subscription.
                outer:
                for (int i = sessions.size() - 1; i >= 0; i--) {
                    List<Subscription> subs = sessions.get(i).subs;
                    for (int j = subs.size() - 1; j >= 0; j--) {
                        Subscription sub = subs.get(j);
                        if (!sub.rate.onChange() && sub.divisor < MAX_DIVISOR) {
                            sub.divisor *= 2;
                            break outer;
                        }
                    }
                }
            } else if (used < budget / 2) {
                outer:
                for (Session s : sessions) {
                    for (Subscription sub : s.subs) {
                        if (sub.divisor > 1) {
                            sub.divisor /= 2;
                            break outer;
                        }
                    }
                }
            }
        }
        windowStart = ts;
        windowBytes = 0;
    }

    private long nextSeq(Session session) {
        long seq = session.seq & 0xFFFFFFFFL;
        session.seq = (session.seq + 1) & 0xFFFFFFFFL;
        return seq;
    }

    // ---- wire helpers ------------------------------------------------------------------------

    /** Writes one framed control message. Marks the session dead on failure. */
    private boolean writeMessage(Session session, ServerMessage message) {
        byte[] framed = ArcpCodec.frame(ArcpCodec.encodeServer(message));
        synchronized (session.writeLock) {
            try {
                session.out.write(framed);
                session.out.flush();
                return true;
            } catch (IOException e) {
                session.dead = true;
                return false;
            }
        }
    }

    /** Length of the body starting at {@code off}, or -1 when incomplete. */
    private static int frameLength(byte[] buf, int off, int available) {
        if (available < 4) {
            return -1;
        }
        long len = (buf[off] & 0xFFL) | ((buf[off + 1] & 0xFFL) << 8) | ((buf[off + 2] & 0xFFL) << 16)
                | ((buf[off + 3] & 0xFFL) << 24);
        if (len > ArcpCodec.MAX_CONTROL_BODY) {
            throw new IllegalStateException("control body too large: " + len);
        }
        return available >= 4 + len ? (int) len : -1;
    }

    private long elapsedMicros() {
        return (System.nanoTime() - startNanos) / 1000L;
    }

    private static void closeQuietly(Socket socket) {
        try {
            socket.close();
        } catch (IOException ignored) {
            // nothing more to do
        }
    }
}
