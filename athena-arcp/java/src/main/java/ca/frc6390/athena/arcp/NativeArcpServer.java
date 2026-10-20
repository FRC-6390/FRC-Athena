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
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * ARCP server for the robot. The protocol is implemented in Rust ({@code arcp-server}); this class
 * only marshals the robot's data across JNI and dispatches client requests to the robot.
 *
 * <p>All methods are called from the robot thread. Clients connect over TCP and UDP on their own
 * threads inside the native server, so nothing here runs on those threads, except that
 * {@link #publishCycle} hands the cycle to a single publisher thread: the native cycle publish writes the
 * realtime frames to the sockets, which would otherwise hold the robot thread for every connected client. Cycles
 * are still delivered in order. The queue holds a few cycles; when it is full the robot thread waits, as it
 * always did, so nothing is dropped. Dictionary, marker and replay publishes wait for queued cycles first.</p>
 *
 * <p>Call {@link #serviceRequests()} once per robot cycle so queued writes, actions and backfill
 * requests are answered. A request left unanswered for the native reply timeout is rejected by the
 * server.</p>
 */
public final class NativeArcpServer implements ArcpSink, AutoCloseable {

    /**
     * Server configuration.
     *
     * @param controlPort TCP control port, 0 for an ephemeral port
     * @param realtimePort UDP port realtime frames are sent from, 0 for an ephemeral port
     * @param fileRoot directory exposed by the file service for plain names, or null
     * @param budgetBytesPerSecond realtime bandwidth budget, 0 for none
     * @param mtu largest UDP datagram in bytes (128 to 65507)
     * @param keyframeIntervalMicros keyframe cadence per subscriber in microseconds
     */
    public record Config(
            int controlPort,
            int realtimePort,
            Path fileRoot,
            long budgetBytesPerSecond,
            int mtu,
            long keyframeIntervalMicros) {

        /** Ephemeral control and realtime ports, no file service, no budget, 1200-byte datagrams, 1 s keyframes. */
        public static Config defaults() {
            return new Config(0, 0, null, 0L, 1200, 1_000_000L);
        }

        public Config {
            if (controlPort < 0 || controlPort > 65_535) {
                throw new IllegalArgumentException("controlPort out of range: " + controlPort);
            }
            if (realtimePort < 0 || realtimePort > 65_535) {
                throw new IllegalArgumentException("realtimePort out of range: " + realtimePort);
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

    private static final int MAX_BACKFILL_BYTES = 1 << 20;

    private final WriteHandler writeHandler;
    private final ActionHandler actionHandler;
    private final Nt4Bridge nt4;
    private ArcpBackfill backfill;

    private final ArcpMarshal.Out dictionaryOut = new ArcpMarshal.Out();
    private final ArcpMarshal.Out cycleOut = new ArcpMarshal.Out();
    private final ArcpMarshal.Out replyOut = new ArcpMarshal.Out();
    private final ArcpMarshal.Out recordsOut = new ArcpMarshal.Out();
    private ByteBuffer requestBuffer = ByteBuffer.allocateDirect(ArcpMarshal.INITIAL_CAPACITY)
            .order(ByteOrder.LITTLE_ENDIAN);

    /** Dictionary entries by signal id, used for NT4 mirroring and for writable-value tracking. */
    private final Map<Long, ArcpDictEntry> dictionary = new HashMap<>();
    /** Last accepted value of each writable signal, reported back when a write handler fails. */
    private final Map<Long, Object> lastWritable = new HashMap<>();
    private final Set<Long> writable = new HashSet<>();

    private long handle;

    /** Cycles handed to the publisher thread that have not been sent yet; a full queue blocks the robot thread. */
    private static final int PUBLISH_QUEUE_CAPACITY = 8;
    private final ThreadPoolExecutor publisher = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(PUBLISH_QUEUE_CAPACITY), NativeArcpServer::publisherThread,
            (task, executor) -> {
                if (executor.isShutdown()) {
                    throw new RejectedExecutionException("ARCP publisher is shut down");
                }
                try {
                    executor.getQueue().put(task);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                    throw new RejectedExecutionException("interrupted while waiting for ARCP publish", interrupted);
                }
            });
    /** Direct buffers the publisher thread has finished with, reused for the next cycle. */
    private final Queue<ByteBuffer> freeCycleBuffers = new ConcurrentLinkedQueue<>();
    /** First failure of a background publish, rethrown on the robot thread at its next call. */
    private final AtomicReference<RuntimeException> publishFailure = new AtomicReference<>();
    /** Session count seen by the last direct status call; read while cycles are still queued. */
    private volatile long lastSessions;

    private static Thread publisherThread(Runnable task) {
        Thread thread = new Thread(task, "athena-arcp-publisher");
        thread.setDaemon(true);
        return thread;
    }

    /** Starts the server with NT4 mirroring disabled and no categories. */
    public NativeArcpServer(Config config, WriteHandler writes, ActionHandler actions) {
        this(config, Map.of(), writes, actions, Nt4Bridge.DISABLED);
    }

    public NativeArcpServer(Config config, WriteHandler writes, ActionHandler actions, Nt4Bridge nt4) {
        this(config, Map.of(), writes, actions, nt4);
    }

    /**
     * Starts the server.
     *
     * @param config server configuration
     * @param fileCategories named directories for the file service; names are {@code category/file}.
     *     Names must be plain words (no slash, colon or tab).
     * @param writes write decisions
     * @param actions action decisions
     * @param nt4 NT4 mirror hook
     */
    public NativeArcpServer(Config config, Map<String, Path> fileCategories, WriteHandler writes,
            ActionHandler actions, Nt4Bridge nt4) {
        Objects.requireNonNull(config, "config");
        this.writeHandler = Objects.requireNonNull(writes, "writes");
        this.actionHandler = Objects.requireNonNull(actions, "actions");
        this.nt4 = Objects.requireNonNull(nt4, "nt4");
        String root = config.fileRoot() == null ? null : config.fileRoot().toAbsolutePath().toString();
        this.handle = ArcpNative.create(config.controlPort(), config.realtimePort(), root,
                config.budgetBytesPerSecond(), config.mtu(),
                config.keyframeIntervalMicros(), categoriesText(fileCategories));
    }

    private static String categoriesText(Map<String, Path> categories) {
        if (categories == null || categories.isEmpty()) {
            return null;
        }
        StringBuilder text = new StringBuilder();
        for (Map.Entry<String, Path> entry : categories.entrySet()) {
            String name = Objects.requireNonNull(entry.getKey(), "category name");
            if (name.isEmpty() || name.matches(".*[/:\\t\\n\\\\].*") || name.equals(".") || name.equals("..")) {
                throw new IllegalArgumentException("invalid file category name: " + name);
            }
            Path dir = Objects.requireNonNull(entry.getValue(), "category directory").toAbsolutePath();
            text.append(name).append('\t').append(dir).append('\n');
        }
        return text.toString();
    }

    /** Port the TCP control listener is bound to. */
    public synchronized int controlPort() {
        return ArcpNative.controlPort(requireOpen());
    }

    /** Local UDP port the realtime frames are sent from. */
    public synchronized int realtimePort() {
        return ArcpNative.realtimePort(requireOpen());
    }

    @Override
    public synchronized void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas) {
        Objects.requireNonNull(entries, "entries");
        Objects.requireNonNull(schemas, "schemas");
        long h = requireOpen();
        awaitQueuedCycles();
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
        rethrowPublishFailure();
        cycleOut.cycle(cycle);
        int length = cycleOut.length();
        ByteBuffer copy = takeCycleBuffer(length);
        copy.put(0, cycleOut.buffer(), 0, length);
        try {
            publisher.execute(() -> publishNative(h, cycleTimestampMicros, copy, length));
        } catch (RejectedExecutionException rejected) {
            recycleCycleBuffer(copy);
            throw rejected;
        }
        if (nt4 != Nt4Bridge.DISABLED) {
            for (int i = 0, n = cycle.size(); i < n; i++) {
                ArcpDictEntry entry = dictionary.get(cycle.signalId(i));
                if (entry != null) {
                    nt4.valueChanged(entry, cycle.value(i));
                }
            }
        }
    }

    @Override
    public synchronized void marker(long timestampMicros, int kind, String message, long signalId) {
        long h = requireOpen();
        awaitQueuedCycles();
        ArcpNative.marker(h, timestampMicros, kind, signalId, message == null ? "" : message);
    }

    @Override
    public synchronized void publishRecords(byte[] records, int length) {
        if (length <= 0) {
            return;
        }
        long h = requireOpen();
        awaitQueuedCycles();
        recordsOut.reset();
        recordsOut.bytes(records, 0, length);
        ArcpNative.publishRecords(h, recordsOut.buffer(), recordsOut.length());
    }

    @Override
    public synchronized void backfill(ArcpBackfill source) {
        this.backfill = source;
    }

    /**
     * Drains client writes, actions and backfill requests, asks the robot, and sends the answers back.
     * Call once per robot cycle. Costs one JNI call when nothing is queued.
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
            } else if (op == ArcpMarshal.OP_BACKFILL) {
                Object last = in.value();
                answerBackfill(signalId, (Long) last);
            } else {
                WriteResult result = decideAction(signalId, op == ArcpMarshal.OP_ACTION_RUN);
                replyOut.u8(result.accepted() ? 1 : 0);
                replyOut.string(result.reason());
            }
        }
        replyOut.patchU32(0, count);
        ArcpNative.reply(h, replyOut.buffer(), replyOut.length());
    }

    private void answerBackfill(long from, long to) {
        if (backfill == null) {
            replyOut.u8(0);
            replyOut.string("no replay buffer");
            replyOut.u32(0);
            return;
        }
        List<byte[]> groups = backfill.records(from, to);
        int total = 0;
        List<byte[]> bounded = new ArrayList<>(groups.size());
        for (byte[] group : groups) {
            if (total + group.length > MAX_BACKFILL_BYTES) {
                break;
            }
            bounded.add(group);
            total += group.length;
        }
        replyOut.u8(1);
        replyOut.string("");
        // The payload is "varint length, group bytes" repeated; Rust forwards it unchanged.
        int payload = 0;
        for (byte[] group : bounded) {
            payload += varintSize(group.length) + group.length;
        }
        replyOut.u32(payload);
        for (byte[] group : bounded) {
            replyOut.varint(group.length);
            replyOut.bytes(group, 0, group.length);
        }
    }

    private static int varintSize(long value) {
        int size = 1;
        while ((value & ~0x7FL) != 0) {
            value >>>= 7;
            size++;
        }
        return size;
    }

    private WriteResult decideWrite(long signalId, Object value) {
        WriteResult result;
        try {
            result = writeHandler.write(signalId, value);
        } catch (RuntimeException ex) {
            return WriteResult.reject("write handler failed: " + ex.getMessage(), held(signalId, value));
        }
        if (result == null) {
            return WriteResult.reject("write handler returned no decision", held(signalId, value));
        }
        if (result.accepted() && writable.contains(signalId)) {
            lastWritable.put(signalId, result.actualValue());
        }
        return result;
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

    private void publishNative(long h, long timestampMicros, ByteBuffer cycle, int length) {
        try {
            ArcpNative.publishCycle(h, timestampMicros, cycle, length);
        } catch (RuntimeException failure) {
            publishFailure.compareAndSet(null, failure);
        } finally {
            recycleCycleBuffer(cycle);
        }
    }

    private void rethrowPublishFailure() {
        RuntimeException failure = publishFailure.getAndSet(null);
        if (failure != null) {
            throw failure;
        }
    }

    /** A direct buffer of at least {@code length} bytes, reused from the publisher thread when possible. */
    private ByteBuffer takeCycleBuffer(int length) {
        ByteBuffer buffer = freeCycleBuffers.poll();
        if (buffer == null || buffer.capacity() < length) {
            buffer = ByteBuffer.allocateDirect(Math.max(length, ArcpMarshal.INITIAL_CAPACITY))
                    .order(ByteOrder.LITTLE_ENDIAN);
        }
        buffer.clear();
        return buffer;
    }

    private void recycleCycleBuffer(ByteBuffer buffer) {
        // Enough buffers for the queue, the one being published and the one being filled.
        if (freeCycleBuffers.size() < PUBLISH_QUEUE_CAPACITY + 2) {
            freeCycleBuffers.add(buffer);
        }
    }

    /** Waits until every cycle handed to the publisher thread has been sent. */
    private void awaitQueuedCycles() {
        rethrowPublishFailure();
        if (publisher.getQueue().isEmpty() && publisher.getActiveCount() == 0) {
            return;
        }
        try {
            publisher.submit(() -> { }).get();
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted while waiting for ARCP publish", interrupted);
        } catch (ExecutionException | RejectedExecutionException failure) {
            throw new IllegalStateException("ARCP publish failed", failure);
        }
        rethrowPublishFailure();
    }

    /**
     * True while at least one client session is connected. Reads the native status directly when no cycle is queued;
     * while cycles are queued the status lock may be held by the publisher, so the last count it saw is used.
     */
    public boolean hasSessions() {
        if (handle == 0) {
            return false;
        }
        if (publisher.getQueue().isEmpty() && publisher.getActiveCount() == 0) {
            synchronized (this) {
                if (handle != 0) {
                    lastSessions = ArcpNative.status(handle)[0];
                }
            }
        }
        return lastSessions > 0;
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
            publisher.shutdown();
            boolean drained;
            try {
                drained = publisher.awaitTermination(5, TimeUnit.SECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                drained = false;
            }
            // A publish still inside the native server must not race the destroy: keep the handle in that case.
            if (drained) {
                ArcpNative.destroy(handle);
            }
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
