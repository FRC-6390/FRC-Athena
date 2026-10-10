package ca.frc6390.athena.arcp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * ARCP server for the robot. The protocol is implemented in Rust ({@code arcp-server}); this class
 * only marshals the robot's data across JNI and dispatches client requests to the robot.
 *
 * <p>All methods are called from the robot thread. Clients connect over TCP and UDP on their own
 * threads inside the native server, so nothing here runs on those threads.</p>
 *
 * <p>Call {@link #serviceRequests()} once per robot cycle so queued writes and actions are answered.
 * A request left unanswered for the native reply timeout is rejected by the server.</p>
 */
public final class NativeArcpServer implements ArcpSink, AutoCloseable {

    /**
     * Server configuration.
     *
     * @param controlPort TCP control port, 0 for an ephemeral port
     * @param fileRoot directory exposed by the file service, or null to disable file requests
     * @param budgetBytesPerSecond realtime bandwidth budget, 0 for none
     * @param mtu largest UDP datagram in bytes (128 to 65507)
     * @param keyframeIntervalMicros keyframe cadence per subscriber in microseconds
     */
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

        public Config {
            if (controlPort < 0 || controlPort > 65_535) {
                throw new IllegalArgumentException("controlPort out of range: " + controlPort);
            }
            if (mtu < 128 || mtu > 65_507) {
                throw new IllegalArgumentException("mtu out of range: " + mtu);
            }
            if (budgetBytesPerSecond < 0) {
                throw new IllegalArgumentException("budgetBytesPerSecond must be >= 0");
            }
            if (keyframeIntervalMicros <= 0) {
                throw new IllegalArgumentException("keyframeIntervalMicros must be > 0");
            }
        }
    }

    private final WriteHandler writeHandler;
    private final ActionHandler actionHandler;
    private final Nt4Bridge nt4;

    private final ArcpMarshal.Out dictionaryOut = new ArcpMarshal.Out();
    private final ArcpMarshal.Out cycleOut = new ArcpMarshal.Out();
    private final ArcpMarshal.Out replyOut = new ArcpMarshal.Out();
    private ByteBuffer requestBuffer = ByteBuffer.allocateDirect(ArcpMarshal.INITIAL_CAPACITY)
            .order(ByteOrder.LITTLE_ENDIAN);

    /** Dictionary entries by signal id, used for NT4 mirroring and for writable-value tracking. */
    private final Map<Long, ArcpDictEntry> dictionary = new HashMap<>();
    /** Latest robot value of each writable signal, reported back when a write cannot be answered. */
    private final Map<Long, Object> lastWritable = new HashMap<>();
    private final Set<Long> writable = new HashSet<>();

    private long handle;

    /** Starts the server with NT4 mirroring disabled. */
    public NativeArcpServer(Config config, WriteHandler writes, ActionHandler actions) {
        this(config, writes, actions, Nt4Bridge.DISABLED);
    }

    public NativeArcpServer(Config config, WriteHandler writes, ActionHandler actions, Nt4Bridge nt4) {
        Objects.requireNonNull(config, "config");
        this.writeHandler = Objects.requireNonNull(writes, "writes");
        this.actionHandler = Objects.requireNonNull(actions, "actions");
        this.nt4 = Objects.requireNonNull(nt4, "nt4");
        String root = config.fileRoot() == null ? null : config.fileRoot().toAbsolutePath().toString();
        this.handle = ArcpNative.create(config.controlPort(), root, config.budgetBytesPerSecond(), config.mtu(),
                config.keyframeIntervalMicros());
    }

    /** Port the TCP control listener is bound to. */
    public synchronized int controlPort() {
        return ArcpNative.controlPort(requireOpen());
    }

    @Override
    public synchronized void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas) {
        Objects.requireNonNull(entries, "entries");
        Objects.requireNonNull(schemas, "schemas");
        long h = requireOpen();
        dictionaryOut.dictionary(entries, schemas);
        ArcpNative.publishDictionary(h, dictionaryOut.buffer(), dictionaryOut.length());

        dictionary.clear();
        writable.clear();
        for (ArcpDictEntry e : entries) {
            dictionary.put(e.signalId(), e);
            if (e.writable()) {
                writable.add(e.signalId());
            }
        }
        lastWritable.keySet().retainAll(writable);
        nt4.dictionaryChanged(entries);
    }

    @Override
    public synchronized void publishCycle(long cycleTimestampMicros, ArcpCycle cycle) {
        Objects.requireNonNull(cycle, "cycle");
        long h = requireOpen();
        cycleOut.cycle(cycle);
        ArcpNative.publishCycle(h, cycleTimestampMicros, cycleOut.buffer(), cycleOut.length());
        boolean mirror = nt4 != Nt4Bridge.DISABLED;
        for (ArcpCycle.Entry e : cycle.entries()) {
            if (writable.contains(e.signalId())) {
                lastWritable.put(e.signalId(), e.value());
            }
            if (mirror) {
                ArcpDictEntry entry = dictionary.get(e.signalId());
                if (entry != null) {
                    nt4.valueChanged(entry, e.value());
                }
            }
        }
    }

    @Override
    public synchronized void marker(long timestampMicros, int kind, String message, long signalId) {
        ArcpNative.marker(requireOpen(), timestampMicros, kind, signalId, message == null ? "" : message);
    }

    /**
     * Drains client writes and actions, asks the handlers, and sends the answers back. Call once per
     * robot cycle. Costs one JNI call when nothing is queued.
     */
    @Override
    public synchronized void serviceRequests() {
        long h = requireOpen();
        int n = ArcpNative.poll(h, requestBuffer, requestBuffer.capacity());
        if (n < 0) {
            requestBuffer = ByteBuffer.allocateDirect(Math.max(-n, requestBuffer.capacity() * 2))
                    .order(ByteOrder.LITTLE_ENDIAN);
            n = ArcpNative.poll(h, requestBuffer, requestBuffer.capacity());
            if (n < 0) {
                throw new IllegalStateException("request buffer still too small after resize");
            }
        }
        ArcpMarshal.In in = new ArcpMarshal.In(requestBuffer);
        if (n < 4) {
            return;
        }
        int count = in.u32();
        if (count == 0) {
            return;
        }
        replyOut.reset();
        replyOut.u32(0);
        for (int i = 0; i < count; i++) {
            long id = in.u64();
            int op = in.u8();
            long signalId = in.u64();
            replyOut.u64(id);
            replyOut.u8(op);
            if (op == ArcpMarshal.OP_WRITE) {
                Object value = in.value();
                WriteResult result = decideWrite(signalId, value);
                replyOut.u8(result.accepted() ? 1 : 0);
                replyOut.string(result.reason());
                replyOut.value(result.actualValue());
            } else {
                WriteResult result = decideAction(signalId, op == ArcpMarshal.OP_ACTION_RUN);
                replyOut.u8(result.accepted() ? 1 : 0);
                replyOut.string(result.reason());
            }
        }
        replyOut.patchU32(0, count);
        ArcpNative.reply(h, replyOut.buffer(), replyOut.length());
    }

    private WriteResult decideWrite(long signalId, Object value) {
        WriteResult result;
        try {
            result = writeHandler.write(signalId, value);
        } catch (RuntimeException ex) {
            return WriteResult.reject("write handler failed: " + ex.getMessage(), held(signalId, value));
        }
        return result == null ? WriteResult.reject("write handler returned no decision", held(signalId, value))
                : result;
    }

    private WriteResult decideAction(long signalId, boolean run) {
        try {
            WriteResult result = actionHandler.action(signalId, run);
            return result == null ? WriteResult.reject("action handler returned no decision", Boolean.FALSE) : result;
        } catch (RuntimeException ex) {
            return WriteResult.reject("action handler failed: " + ex.getMessage(), Boolean.FALSE);
        }
    }

    private Object held(long signalId, Object requested) {
        Object last = lastWritable.get(signalId);
        return last != null ? last : requested;
    }

    @Override
    public synchronized ArcpStatus status() {
        if (handle == 0) {
            return ArcpStatus.STOPPED;
        }
        long[] s = ArcpNative.status(handle);
        String throttled = ArcpNative.throttled(handle);
        List<String> lines = new ArrayList<>();
        if (throttled != null) {
            for (String line : throttled.split("\n")) {
                if (!line.isBlank()) {
                    lines.add(line);
                }
            }
        }
        return new ArcpStatus(s[0], s[1], s[2], lines);
    }

    /** Stops accepting clients and frees the native server. Safe to call twice. */
    @Override
    public synchronized void close() {
        if (handle != 0) {
            ArcpNative.destroy(handle);
            handle = 0;
        }
    }

    private long requireOpen() {
        if (handle == 0) {
            throw new IllegalStateException("NativeArcpServer is closed");
        }
        return handle;
    }
}
