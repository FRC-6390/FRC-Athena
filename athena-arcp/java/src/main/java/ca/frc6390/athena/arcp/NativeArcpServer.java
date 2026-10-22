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
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * ARCP server for the robot. The protocol is implemented in Rust ({@code arcp-server}); this class
 * only marshals the robot's data across JNI and dispatches client requests to the robot.
 *
 * <p>Robot publication methods and the replay encoder admit operations through this synchronized facade.
 * Clients connect over TCP and UDP on native threads. A single publisher thread performs ordered JNI work: the native cycle publish writes the
 * realtime frames to the sockets, which would otherwise hold the robot thread for every connected client. Cycles
 * are delivered in order. When the bounded queue fills, the newest pending cycle absorbs the latest signal
 * values instead of delaying the robot; intermediate telemetry samples may be coalesced. Dictionary, marker and
 * replay publishes share the same bounded ordered mailbox and never wait for queued cycles.</p>
 *
 * <p>Call {@link #serviceRequests()} once per robot cycle so queued writes and actions are answered and
 * history requests are handed to the bounded backfill worker. A request left unanswered for the native reply timeout is rejected by the
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

    // The wire body also includes kind, request id, reply status, data tag, and record-list length.
    private static final int MAX_BACKFILL_BYTES = (1 << 20) - 64;
    private static final long BACKFILL_QUEUE_DEADLINE_NANOS = TimeUnit.MILLISECONDS.toNanos(200);

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

    /** Bounded ordered publication; adjacent cycle deltas coalesce without crossing protocol operations. */
    private final CyclePublisher publisher = new CyclePublisher(ArcpNative::publishCycle);
    /** At most one history response is materialized, plus two lightweight waiting request descriptors. */
    private final ThreadPoolExecutor backfills = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(2), runnable -> {
                Thread thread = new Thread(runnable, "athena-arcp-backfill");
                thread.setDaemon(true);
                thread.setPriority(Thread.MIN_PRIORITY);
                return thread;
            });
    private final ArcpMarshal.Out backfillOut = new ArcpMarshal.Out(); // Owned only by the backfill worker.
    private final AtomicReference<RuntimeException> backfillFailure = new AtomicReference<>();
    /** Session count seen by the last direct status call; read while cycles are still queued. */
    private volatile long lastSessions;
    private volatile ArcpStatus cachedStatus = ArcpStatus.STOPPED;
    private final java.util.concurrent.atomic.AtomicBoolean statusPending = new java.util.concurrent.atomic.AtomicBoolean();
    private long lastStatusRequest;
    private volatile long lastServiceNanos;
    private volatile long maxCallbackNanos;
    private volatile long rejectedCallbacks;
    private static final long REQUEST_BUDGET_NANOS = 1_000_000;
    private static final int MAX_REQUEST_BUFFER_BYTES = 2 * 1024 * 1024;

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
        dictionaryOut.dictionary(entries, schemas);
        publisher.execute(dictionaryOut.buffer(), dictionaryOut.length(),
                (bytes, length) -> ArcpNative.publishDictionary(h, bytes, length));

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
        publisher.publish(h, cycleTimestampMicros, cycleOut.buffer(), cycleOut.length());
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
        String text = message == null ? "" : message;
        publisher.execute(() -> ArcpNative.marker(h, timestampMicros, kind, signalId, text),
                Math.addExact(64, Math.multiplyExact(text.length(), 2)));
    }

    @Override
    public synchronized void publishRecords(byte[] records, int length) {
        if (length <= 0) {
            return;
        }
        long h = requireOpen();
        recordsOut.reset();
        recordsOut.bytes(records, 0, length);
        publisher.execute(recordsOut.buffer(), recordsOut.length(),
                (bytes, size) -> ArcpNative.publishRecords(h, bytes, size));
    }

    @Override
    public synchronized void backfill(ArcpBackfill source) {
        this.backfill = source;
    }

    /**
     * Services at most eight client requests, with a 1 ms admission budget between callbacks, on the robot thread and dispatches history requests to a bounded worker.
     * Backfill materialization and reply copying never run on this thread. Costs one JNI call when idle.
     */
    @Override
    public synchronized void serviceRequests() {
        long serviceStart = System.nanoTime();
        long deadline = serviceStart + REQUEST_BUDGET_NANOS;
        long h = requireOpen();
        scheduleStatus();
        RuntimeException failedBackfill = backfillFailure.getAndSet(null);
        if (failedBackfill != null) throw failedBackfill;
        int n = ArcpNative.poll(h, requestBuffer, requestBuffer.capacity());
        if (n < 0) {
            if (-n > MAX_REQUEST_BUFFER_BYTES) throw new IllegalStateException("ARCP request exceeds buffer limit");
            requestBuffer = ByteBuffer.allocateDirect(Math.min(MAX_REQUEST_BUFFER_BYTES, Math.max(-n, requestBuffer.capacity() * 2)))
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
        int immediateReplies = 0;
        int dispatched = 0;
        for (int i = 0; i < count; i++) {
            long id = in.u64();
            int op = in.u8();
            long signalId = in.u64();
            if (op == ArcpMarshal.OP_BACKFILL) {
                long last = (Long) in.value();
                String rejected = queueBackfill(h, id, signalId, last);
                if (rejected == null) continue;
                replyOut.u64(id);
                replyOut.u8(op);
                backfillError(replyOut, rejected);
                immediateReplies++;
                continue;
            }
            replyOut.u64(id);
            replyOut.u8(op);
            if (op == ArcpMarshal.OP_WRITE) {
                Object value = in.value();
                long before = System.nanoTime();
                WriteResult result;
                if (dispatched > 0 && before >= deadline) {
                    rejectedCallbacks++;
                    result = WriteResult.reject("robot request service budget exhausted", held(signalId, value));
                } else {
                    result = decideWrite(signalId, value);
                    dispatched++;
                    maxCallbackNanos = Math.max(maxCallbackNanos, System.nanoTime() - before);
                }
                replyOut.u8(result.accepted() ? 1 : 0);
                replyOut.string(result.reason());
                replyOut.value(result.actualValue());
            } else {
                long before = System.nanoTime();
                WriteResult result;
                if (dispatched > 0 && before >= deadline) {
                    rejectedCallbacks++;
                    result = WriteResult.reject("robot request service budget exhausted", Boolean.FALSE);
                } else {
                    result = decideAction(signalId, op == ArcpMarshal.OP_ACTION_RUN);
                    dispatched++;
                    maxCallbackNanos = Math.max(maxCallbackNanos, System.nanoTime() - before);
                }
                replyOut.u8(result.accepted() ? 1 : 0);
                replyOut.string(result.reason());
            }
            immediateReplies++;
        }
        if (immediateReplies > 0) {
            replyOut.patchU32(0, immediateReplies);
            ArcpNative.reply(h, replyOut.buffer(), replyOut.length());
        }
        lastServiceNanos = System.nanoTime() - serviceStart;
    }

    /** Returns a rejection reason, or null after a request was accepted by the bounded worker. */
    private String queueBackfill(long handle, long id, long from, long to) {
        ArcpBackfill source = backfill;
        if (source == null) return "no replay buffer";
        if (from < 0 || to < from) return "invalid replay sequence range";
        long submitted = System.nanoTime();
        try {
            backfills.execute(() -> answerBackfill(handle, id, source, from, to, submitted));
            return null;
        } catch (RejectedExecutionException rejected) {
            return "replay backfill busy: request queue full or closed";
        }
    }

    /** Runs entirely on the worker; no robot-thread marshal buffers or hardware suppliers are used here. */
    private void answerBackfill(long handle, long id, ArcpBackfill source, long from, long to, long submitted) {
        backfillOut.reset();
        backfillOut.u32(1);
        backfillOut.u64(id);
        backfillOut.u8(ArcpMarshal.OP_BACKFILL);
        try {
            if (System.nanoTime() - submitted >= BACKFILL_QUEUE_DEADLINE_NANOS) {
                backfillError(backfillOut, "replay backfill expired while queued");
            } else {
                // The built-in history provider selects before copying. Validate custom providers too, and
                // account for every group prefix before growing the response buffer or copying its bytes.
                List<byte[]> groups = ArcpBackfill.bounded(source.records(from, to, MAX_BACKFILL_BYTES),
                        MAX_BACKFILL_BYTES);
                int payload = 0;
                for (byte[] group : groups) payload += varintSize(group.length) + group.length;
                backfillOut.u8(1);
                backfillOut.string("");
                backfillOut.u32(payload);
                for (byte[] group : groups) {
                    backfillOut.varint(group.length);
                    backfillOut.bytes(group, 0, group.length);
                }
            }
        } catch (RuntimeException failure) {
            // Keep the request id/op, replacing any partial success payload with an explicit rejection.
            backfillOut.reset();
            backfillOut.u32(1);
            backfillOut.u64(id);
            backfillOut.u8(ArcpMarshal.OP_BACKFILL);
            backfillError(backfillOut, "replay backfill failed: " + failure.getMessage());
        }
        try {
            ArcpNative.reply(handle, backfillOut.buffer(), backfillOut.length());
        } catch (RuntimeException failure) {
            backfillFailure.compareAndSet(null, new IllegalStateException("ARCP backfill reply failed", failure));
        }
    }

    private static void backfillError(ArcpMarshal.Out out, String reason) {
        out.u8(0);
        out.string(reason.length() <= 512 ? reason : reason.substring(0, 512));
        out.u32(0);
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

    /**
     * True when the most recent background status sample contained a connected client session.
     * This never reads native state synchronously on the caller.
     */
    public boolean hasSessions() { return status().sessions() > 0; }

    /** Cached background sample; never competes for native publication/state locks on the robot thread. */
    @Override
    public synchronized ArcpStatus status() {
        if (handle == 0) return ArcpStatus.STOPPED;
        scheduleStatus();
        return cachedStatus;
    }

    private void scheduleStatus() {
        long now = System.nanoTime();
        if (now - lastStatusRequest < 100_000_000 || !statusPending.compareAndSet(false, true)) return;
        long h = handle;
        boolean accepted = publisher.tryExecute(() -> {
            try {
                long[] s = ArcpNative.status(h);
                String throttled = ArcpNative.throttled(h);
                List<String> lines = new ArrayList<>();
                if (throttled != null) for (String line : throttled.split("\n")) {
                    if (!line.isBlank()) lines.add(line);
                }
                lastSessions = s[0];
                cachedStatus = new ArcpStatus(s[0], s[1], s[2], lines);
            } finally { statusPending.set(false); }
        });
        if (accepted) lastStatusRequest = now;
        else statusPending.set(false);
    }

    /** Explicit status for an overloaded or failed ordered publisher. */
    public String publicationFailure() { return publisher.failureMessage(); }
    /** Last non-idle service duration; arbitrary user callbacks remain non-preemptible. */
    public long lastServiceNanos() { return lastServiceNanos; }
    public long maxCallbackNanos() { return maxCallbackNanos; }
    public long rejectedCallbacks() { return rejectedCallbacks; }

    /** Stops accepting work and frees the native server after its workers finish. Safe to call twice. */
    @Override
    public void close() {
        long closingHandle;
        synchronized (this) {
            if (handle == 0) return;
            closingHandle = handle;
            handle = 0;
            backfills.shutdown();
        }
        // A worker may briefly need the recorder monitor while a lifecycle marker calls back into this
        // server. Never hold the server monitor while joining that worker.
        boolean backfillsDrained;
        try {
            backfillsDrained = backfills.awaitTermination(5, TimeUnit.SECONDS);
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            backfillsDrained = false;
        }
        boolean drained = publisher.close();
        // A publish or backfill still using the native server must not race its destruction.
        if (drained && backfillsDrained) ArcpNative.destroy(closingHandle);
        if (!backfillsDrained) {
            backfills.shutdownNow();
            throw new IllegalStateException("ARCP backfill worker did not stop; native server retained safely.");
        }
    }

    private long requireOpen() {
        if (handle == 0) {
            throw new IllegalStateException("NativeArcpServer is closed");
        }
        return handle;
    }
}
