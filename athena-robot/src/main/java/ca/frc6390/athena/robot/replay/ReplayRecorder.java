package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeFailureReporter;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.time.LocalDateTime;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Records every runtime cycle into an in-RAM buffer, flushes buffered data to {@code .athrp} files on
 * replay events, and optionally streams live records to a {@link ReplayStreamSink}.
 *
 * <p>Attached to a runtime with {@code RobotRuntime.replay(...)}. All methods run on the robot thread, except
 * {@link #mark}, {@link #keyframe}, and the waiting and counting accessors. A flush does not compress or touch
 * the disk on the robot thread: it hands the buffered segments to a {@link ReplayWriter} thread, see
 * {@link #flush()}.</p>
 *
 * <p>Every encoding, marker, lifecycle, keyframe, streaming and flush operation runs on one low-priority
 * ordered worker. Producers only admit immutable descriptors. Its 64-cycle backlog drops oldest pending
 * cycles explicitly when overloaded; a separate 32-command reserve preserves critical ordering or rejects
 * admission with an exception. Encoded ring history and disk jobs share the configured history byte budget;
 * immutable cycle descriptors have a separate bounded count.</p>
 */
public final class ReplayRecorder {
    /** Segments close at a cycle boundary once they reach this size. */
    static final int SEGMENT_TARGET_BYTES = 64 * 1024;
    /** Maximum cycle descriptors waiting for the encoder thread. */
    static final int MAX_PENDING_CYCLES = 64;
    private static final long ENCODER_POLL_MILLIS = 40;
    private static final int ENCODER_IDLE_POLLS = 50;

    private final ReplayPolicy policy;
    private final Path directory;
    private final RuntimeFailureReporter reporter;
    private final ReplayEncoder encoder = new ReplayEncoder();
    private final ReplayBuffer buffer;
    private final WireWriter dictionaryScratch = new WireWriter(512);
    private final WireWriter frameScratch = new WireWriter(2048);
    private final WireWriter markerScratch = new WireWriter(256);
    private Supplier<LocalDateTime> clock = LocalDateTime::now;

    private WireWriter segment;
    /** Closed segment buffer kept for reuse; a closed segment is copied out, so its writer can be cleared. */
    private WireWriter spareSegment;
    private int segmentFrames;
    private long segmentFirstSequence = -1;
    private long segmentLastSequence = -1;
    private double segmentFirstTimestamp;
    private double segmentLastTimestamp;
    // Volatile so the accessors do not wait for the encoder thread; written under this recorder's monitor.
    private volatile ReplayMatch match = ReplayMatch.none();
    private boolean enabledSinceFlush;
    private long lastSequence = -1;
    private double lastTimestampSeconds;
    private volatile ReplayStreamSink sink;
    private volatile ReplayStreamSink requestedSink;
    private final Set<String> issuedNames = new HashSet<>();
    private final List<MarkerListener> markerListeners = new java.util.concurrent.CopyOnWriteArrayList<>();
    private volatile ReplayWriter writer;
    private volatile long recordingEvictions;
    private final long sessionId = SESSIONS.incrementAndGet();

    // Producers only hold this admission monitor; encoding and callbacks never do.
    private final ArrayDeque<Operation> pending = new ArrayDeque<>();
    private static final int MAX_PENDING_COMMANDS = 32;
    private Thread encoderThread;
    private boolean encoderStopped;
    private boolean workerStopped;
    private int pendingCycles;
    private long submitted;
    private long completed;
    private volatile long droppedCycles;
    private volatile long rejectedCommands;
    private volatile long failedCommands;
    private volatile String lastFailure = "";
    private long reportedDrops;
    private long acceptedSequence = -1;
    private double acceptedTimestampSeconds;
    private boolean dirty;
    private record Operation(long ticket, boolean cycle, Runnable work) { }

    /** Session identities are unique within a JVM and increase over time (seeded from the wall clock). */
    private static final java.util.concurrent.atomic.AtomicLong SESSIONS =
            new java.util.concurrent.atomic.AtomicLong(System.currentTimeMillis() / 1000L * 1000L);

    /** Receives every marker as it is recorded, for live forwarding (for example to ARCP clients). */
    @FunctionalInterface
    public interface MarkerListener {
        /**
         * @param timestampSeconds robot time of the latest recorded cycle
         * @param kind marker kind, see {@link ReplayMarkers}
         * @param message marker message
         * @param signalId related signal id, or 0
         */
        void marker(double timestampSeconds, int kind, String message, int signalId);
    }

    /**
     * Adds a listener that is called for each marker recorded from now on. Listeners run on the ordered replay worker and must not block.
     *
     * @param listener listener
     */
    public void addMarkerListener(MarkerListener listener) {
        markerListeners.add(java.util.Objects.requireNonNull(listener, "listener"));
    }

    public ReplayRecorder(ReplayPolicy policy, Path directory, RuntimeFailureReporter reporter) {
        this.policy = policy;
        this.directory = directory;
        this.reporter = reporter == null ? RuntimeFailureReporter.stderr() : reporter;
        this.buffer = new ReplayBuffer(policy.bufferBytes());
    }

    /**
     * Returns the identity of this recording session. It is fixed for the life of the recorder and is reported as
     * the replay id of recorded interpolation points.
     *
     * @return session identity
     */
    public long sessionId() {
        return sessionId;
    }

    /** Returns the sequence number of the latest recorded cycle (the frame index), or -1 before the first cycle. */
    public long currentFrameIndex() {
        synchronized (pending) {
            return acceptedSequence;
        }
    }

    /** Returns the timestamp of the latest recorded cycle, in seconds. */
    public double currentTimestampSeconds() {
        synchronized (pending) {
            return acceptedTimestampSeconds;
        }
    }

    /** Returns the policy in effect. */
    public ReplayPolicy policy() {
        return policy;
    }

    /** Returns the directory flushed files are written to. */
    public Path directory() {
        return directory;
    }

    /** Returns the most recent file the writer thread finished. */
    public Optional<Path> lastFile() {
        ReplayWriter current = writer;
        return current == null ? Optional.empty() : current.lastWritten();
    }

    /**
     * Waits until every queued flush has been written, or dropped. Intended for tests and orderly shutdown; the
     * robot loop must not call it.
     *
     * @param timeout maximum wait
     * @return true when no flush is pending
     * @throws InterruptedException when interrupted while waiting
     */
    public boolean awaitFlushed(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        if (!awaitEncoded(timeout)) return false;
        ReplayWriter current = writer;
        return current == null || current.awaitIdle(Duration.ofNanos(Math.max(0, deadline - System.nanoTime())));
    }

    /** Returns flushes accepted but not yet written. */
    public int pendingFlushes() {
        ReplayWriter current = writer;
        return current == null ? 0 : current.pending();
    }

    /** Returns flushes dropped because the writer queue was full. */
    public long droppedFlushes() {
        ReplayWriter current = writer;
        return current == null ? 0 : current.dropped();
    }

    /** Bytes held by the disk writer's current and queued immutable snapshots. */
    public long pendingReplayBytes() {
        ReplayWriter current = writer;
        return current == null ? 0 : current.retainedBytes();
    }

    /** Segments evicted from the live ring because an unfinished disk write shares its history budget. */
    public long recordingEvictions() {
        return recordingEvictions;
    }

    /** Returns the match identity recorded in new files. */
    public ReplayMatch match() {
        return match;
    }

    /**
     * Installs a stream sink. Streaming happens only when the policy enables it.
     *
     * @param sink sink, or null to remove it
     */
    public void streamSink(ReplayStreamSink sink) {
        synchronized (pending) {
            enqueue(false, () -> {
            this.sink = sink;
            if (sink != null) sink.attach(new ReplayStreamSink.ReplayBackfill() {
                @Override public List<byte[]> records(long from, long to) { return backfill(from, to); }
                @Override public List<byte[]> records(long from, long to, int maxBytes) {
                    return backfill(from, to, maxBytes);
                }
            });
            });
            requestedSink = sink;
        }
    }

    /** Returns true when frames are being published to a sink. */
    public boolean streaming() {
        return policy.streamEnabled() && requestedSink != null;
    }

    private boolean streamingActive() {
        return policy.streamEnabled() && sink != null;
    }

    /**
     * Records one cycle.
     *
     * @param snapshot runtime cycle snapshot
     * @param mechanismInputs hardware inputs the mechanisms used, or null to use the snapshot's
     * @param outputs resolved mechanism outputs, or null for cycles that ran no mechanisms
     */
    public void record(RuntimeCycleSnapshot snapshot, HardwareCycleSnapshot mechanismInputs,
                       List<ResolvedOutput> outputs) {
        Objects.requireNonNull(snapshot, "snapshot");
        List<ResolvedOutput> immutableOutputs = outputs == null ? null : List.copyOf(outputs);
        synchronized (pending) {
            if (encoderStopped) return;
            enqueueLocked(true, () -> encode(snapshot, mechanismInputs, immutableOutputs));
            acceptedSequence = snapshot.sequence();
            acceptedTimestampSeconds = snapshot.timestampSeconds();
            dirty = true;
        }
    }

    /** Cycles discarded while the encoder could not keep up. Sequence gaps remain visible to readers. */
    public long droppedCycles() { return droppedCycles; }
    /** Critical commands explicitly refused when their separate bounded reserve was exhausted. */
    public long rejectedCommands() { return rejectedCommands; }
    public long failedCommands() { return failedCommands; }
    public String lastFailure() { return lastFailure; }

    private void enqueue(boolean cycle, Runnable work) {
        synchronized (pending) {
            if (encoderStopped) throw new IllegalStateException("Replay recorder is closed");
            enqueueLocked(cycle, work);
        }
    }

    private void enqueueLocked(boolean cycle, Runnable work) {
        if (encoderStopped) throw new IllegalStateException("Replay recorder is closed");
        if (cycle && pendingCycles >= MAX_PENDING_CYCLES) {
            var iterator = pending.iterator();
            while (iterator.hasNext()) {
                if (iterator.next().cycle()) { iterator.remove(); pendingCycles--; droppedCycles++; break; }
            }
        }
        if (!cycle && pending.size() - pendingCycles >= MAX_PENDING_COMMANDS) {
            rejectedCommands++;
            throw new java.util.concurrent.RejectedExecutionException("Replay critical command queue full");
        }
        pending.addLast(new Operation(++submitted, cycle, work));
        if (cycle) pendingCycles++;
        startEncoder();
        pending.notifyAll();
    }

    private void startEncoder() {
        if (encoderThread != null) return;
        encoderThread = new Thread(this::encodeQueued, "athena-replay-encoder");
        encoderThread.setDaemon(true);
        encoderThread.setPriority(Thread.MIN_PRIORITY);
        encoderThread.start();
    }

    private void encodeQueued() {
        int idlePolls = 0;
        while (true) {
            Operation next;
            synchronized (pending) {
                while (pending.isEmpty() && !encoderStopped) {
                    try { pending.wait(ENCODER_POLL_MILLIS); }
                    catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return; }
                    if (pending.isEmpty() && !encoderStopped && ++idlePolls >= ENCODER_IDLE_POLLS) {
                        encoderThread = null;
                        return;
                    }
                }
                if (pending.isEmpty()) break;
                next = pending.removeFirst();
                idlePolls = 0;
                if (next.cycle()) pendingCycles--;
            }
            try {
                long lost = droppedCycles - reportedDrops;
                if (lost > 0) {
                    reportedDrops += lost;
                    encoder.requestKeyframe();
                    try {
                        markEncoded(ReplayMarkers.FAULT, "replay encoder dropped " + lost + " cycle(s)", 0);
                        reporter.warning("Athena replay encoder overload; dropped " + lost + " cycle(s).", null);
                    } catch (Throwable reportingFailure) {
                        noteFailure(next.ticket(), reportingFailure);
                    }
                }
                next.work().run();
            } catch (Throwable failure) {
                noteFailure(next.ticket(), failure);
            } finally {
                synchronized (pending) { completed = next.ticket(); pending.notifyAll(); }
            }
        }
        sink = null;
        ReplayWriter current = writer;
        if (current != null) current.close();
        synchronized (pending) { workerStopped = true; pending.notifyAll(); }
    }

    private void noteFailure(long ticket, Throwable failure) {
        failedCommands++;
        lastFailure = failure.getClass().getSimpleName() + ": " + failure.getMessage();
        encoder.requestKeyframe();
        try { reporter.error("Athena replay command " + ticket + " failed.", failure); }
        catch (Throwable reportingFailure) {
            lastFailure += "; reporter failed: " + reportingFailure.getMessage();
        }
    }

    /** Explicit test/shutdown fence; periodic and lifecycle methods never call this. */
    public boolean awaitEncoded(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        synchronized (pending) {
            long through = submitted;
            while (completed < through) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return false;
                java.util.concurrent.TimeUnit.NANOSECONDS.timedWait(pending, remaining);
            }
            return true;
        }
    }

    /** Waits only during explicit orderly shutdown. */
    public boolean awaitStopped(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        synchronized (pending) {
            while (!workerStopped) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return false;
                java.util.concurrent.TimeUnit.NANOSECONDS.timedWait(pending, remaining);
            }
            return true;
        }
    }

    private void encode(
            RuntimeCycleSnapshot snapshot,
            HardwareCycleSnapshot mechanismInputs,
            List<ResolvedOutput> outputs) {
        if (segment == null || segment.size() >= SEGMENT_TARGET_BYTES) {
            closeSegment();
            openSegment();
            // openSegment wrote the dictionary of every known signal into dictionaryScratch. Stream it now, or live
            // clients would receive values without names until a new signal appears.
            if (streamingActive() && dictionaryScratch.size() > 0) {
                sink.publishDictionary(dictionaryScratch.toByteArray());
            }
        }
        dictionaryScratch.clear();
        frameScratch.clear();
        encoder.encode(snapshot, mechanismInputs, outputs, dictionaryScratch, frameScratch);
        segment.bytes(dictionaryScratch.buffer(), 0, dictionaryScratch.size());
        segment.bytes(frameScratch.buffer(), 0, frameScratch.size());
        segmentFrames++;
        lastSequence = snapshot.sequence();
        lastTimestampSeconds = snapshot.timestampSeconds();
        if (segmentFirstSequence < 0) {
            segmentFirstSequence = lastSequence;
            segmentFirstTimestamp = lastTimestampSeconds;
        }
        segmentLastSequence = lastSequence;
        segmentLastTimestamp = lastTimestampSeconds;
        if (streamingActive()) {
            if (dictionaryScratch.size() > 0) {
                sink.publishDictionary(dictionaryScratch.toByteArray());
            }
            sink.publishFrame(lastSequence, frameScratch.toByteArray());
        }
    }

    /**
     * Adds a marker at the latest cycle time.
     *
     * @param kind marker kind, see {@link ReplayMarkers}
     * @param message marker message
     */
    public void mark(int kind, String message) { mark(kind, message, 0); }

    public void mark(int kind, String message, int signalId) {
        enqueue(false, () -> markEncoded(kind, message, signalId));
    }

    private void markEncoded(int kind, String message, int signalId) {
        if (segment == null) openSegment();
        markerScratch.clear();
        encoder.writeMarker(markerScratch, lastTimestampSeconds, kind, message, signalId);
        segment.bytes(markerScratch.buffer(), 0, markerScratch.size());
        if (streamingActive()) sink.publishMarker(Math.max(0, lastSequence), markerScratch.toByteArray());
        for (MarkerListener listener : markerListeners) listener.marker(lastTimestampSeconds, kind, message, signalId);
    }

    /** Forces the next accepted cycle after this command to contain every known signal. */
    public void keyframe() { enqueue(false, encoder::requestKeyframe); }

    /**
     * Reserves a destination and queues a flush without encoding or waiting. The returned path is a request,
     * not a durability acknowledgement: writer refusals/failures are reported asynchronously. Use
     * awaitFlushed only from tests or orderly shutdown.
     */
    public Optional<Path> flush() {
        synchronized (pending) {
            if (encoderStopped || !dirty) return Optional.empty();
            ReplayMatch identity = match;
            LocalDateTime now = clock.get();
            Path target = reserveTarget(identity, now);
            enqueueLocked(false, () -> flushEncoded("explicit", target, identity, now));
            dirty = false;
            return Optional.of(target);
        }
    }

    public void matchIdentified(ReplayMatch identified) {
        if (identified == null || !identified.identified()) return;
        synchronized (pending) {
            if (identified.equals(match)) return;
            ReplayMatch previous = match;
            Runnable flush = flushCommand("match-identified", previous);
            enqueueLocked(false, () -> {
                flush.run();
                markEncoded(ReplayMarkers.MATCH_IDENTIFIED, identified.eventName() + " "
                        + identified.typeCode() + identified.matchNumber(), 0);
            });
            match = identified;
        }
    }

    public void enabledPeriod(int markerKind, String message) {
        enqueue(false, () -> { enabledSinceFlush = true; markEncoded(markerKind, message, 0); });
    }

    public void matchEnd() {
        Runnable flush = flushCommand("match-end", match);
        enqueue(false, () -> {
            markEncoded(ReplayMarkers.MATCH_END, "teleop ended", 0);
            enabledSinceFlush = false;
            if (policy.flushes(ReplayPolicy.Flush.MATCH_END)) flush.run();
        });
    }

    public void disabled() {
        Runnable flush = flushCommand("disable-after-match", match);
        enqueue(false, () -> {
            markEncoded(ReplayMarkers.DISABLE, "disabled", 0);
            boolean afterMatch = enabledSinceFlush;
            enabledSinceFlush = false;
            if (afterMatch && policy.flushes(ReplayPolicy.Flush.DISABLE_AFTER_MATCH)) {
                flush.run();
            }
        });
    }

    public void brownoutRecovered() {
        Runnable flush = flushCommand("brownout-recovery", match);
        enqueue(false, () -> {
            markEncoded(ReplayMarkers.BROWNOUT, "brownout recovered", 0);
            if (policy.flushes(ReplayPolicy.Flush.BROWNOUT_RECOVERY)) flush.run();
        });
    }

    /** Stops admission immediately; accepted commands drain in order before the writer is closed. */
    public void close() {
        synchronized (pending) {
            if (encoderStopped) return;
            encoderStopped = true;
            requestedSink = null;
            if (encoderThread == null) {
                sink = null;
                if (writer != null) writer.close();
                workerStopped = true;
                return;
            }
            pending.notifyAll();
        }
    }

    void clock(Supplier<LocalDateTime> clock) {
        this.clock = clock;
    }

    private Runnable flushCommand(String reason, ReplayMatch identity) {
        LocalDateTime now = clock.get();
        Path target = reserveTarget(identity, now);
        return () -> flushEncoded(reason, target, identity, now);
    }

    private Optional<Path> flushEncoded(String reason, Path target, ReplayMatch identity, LocalDateTime now) {
        closeSegment();
        List<ReplayBuffer.Segment> segments = buffer.drain();
        if (segments.isEmpty()) {
            return Optional.empty();
        }
        List<byte[]> records = new ArrayList<>(segments.size());
        for (ReplayBuffer.Segment segment : segments) {
            records.add(segment.records());
        }
        WireWriter dictionary = new WireWriter(1024);
        encoder.writeDictionary(dictionary);
        Map<String, String> metadata = new LinkedHashMap<>();
        metadata.put("format", "athena-replay");
        metadata.put("formatVersion", String.valueOf(ca.frc6390.athena.runtime.wire.Wire.REPLAY_FORMAT_VERSION));
        metadata.put("reason", reason);
        metadata.put("event", identity.eventName());
        metadata.put("matchType", identity.matchType());
        metadata.put("matchNumber", String.valueOf(identity.matchNumber()));
        metadata.put("identified", String.valueOf(identity.identified()));
        metadata.put("firstSequence", String.valueOf(segments.get(0).firstSequence()));
        metadata.put("lastSequence", String.valueOf(segments.get(segments.size() - 1).lastSequence()));
        metadata.put("firstTimestampSeconds", String.valueOf(segments.get(0).firstTimestampSeconds()));
        metadata.put("lastTimestampSeconds",
                String.valueOf(segments.get(segments.size() - 1).lastTimestampSeconds()));
        metadata.put("createdAt", now.toString());
        if (!replayWriter().submit(new ReplayWriter.Job(target, metadata, dictionary.toByteArray(), records, reason))) {
            // Keep the newest bounded history available for retry/backfill if the current write occupies the budget.
            for (ReplayBuffer.Segment retained : segments) retainBuffered(retained);
            synchronized (pending) { dirty = true; }
            reporter.error("Athena replay flush to " + target
                    + " was refused; writer closed or memory budget exhausted. Newest data remains in RAM.", null);
            return Optional.empty();
        }
        return Optional.of(target);
    }

    private Path reserveTarget(ReplayMatch identity, LocalDateTime now) {
        synchronized (issuedNames) {
            String name = ReplayFiles.fileName(identity, now);
            for (int attempt = 0; issuedNames.contains(name); attempt++) {
                now = now.plusSeconds(1);
                name = ReplayFiles.fileName(identity, now);
            }
            issuedNames.add(name);
            return directory.resolve(name);
        }
    }

    private ReplayWriter replayWriter() {
        if (writer == null) {
            writer = new ReplayWriter(reporter, protectedName -> ReplayFiles.at(directory)
                    .enforce(policy.retention(), protectedName), job -> { },
                    Math.max(SEGMENT_TARGET_BYTES, policy.bufferBytes()) + 1024 * 1024);
        }
        return writer;
    }

    private void openSegment() {
        if (spareSegment != null) {
            segment = spareSegment.clear();
            spareSegment = null;
        } else {
            segment = new WireWriter(SEGMENT_TARGET_BYTES + 8192);
        }
        segmentFrames = 0;
        segmentFirstSequence = -1;
        segmentLastSequence = -1;
        encoder.requestKeyframe();
        dictionaryScratch.clear();
        encoder.writeDictionary(dictionaryScratch);
        segment.bytes(dictionaryScratch.buffer(), 0, dictionaryScratch.size());
    }

    private void closeSegment() {
        if (segment == null) {
            return;
        }
        if (segment.size() > 0 && segmentFrames > 0) {
            retainBuffered(new ReplayBuffer.Segment(
                    segment.toByteArray(), segmentFirstSequence, segmentLastSequence,
                    segmentFirstTimestamp, segmentLastTimestamp));
        }
        spareSegment = segment;
        segment = null;
    }

    private void retainBuffered(ReplayBuffer.Segment retainedSegment) {
        long retained = pendingReplayBytes();
        long before = buffer.evictedSegments();
        buffer.add(retainedSegment, Math.max(0, policy.bufferBytes() - retained));
        if (retained > 0) {
            long evicted = buffer.evictedSegments() - before;
            recordingEvictions += evicted;
            writer.recordingEvicted(evicted);
        }
    }

    /** Returns a bounded snapshot; bulk closed-segment copies never hold the recorder's monitor. */
    List<byte[]> backfill(long fromSequence, long toSequence) {
        return backfill(fromSequence, toSequence, ReplayStreamSink.DEFAULT_BACKFILL_BYTES);
    }

    List<byte[]> backfill(long fromSequence, long toSequence, int maxBytes) {
        if (fromSequence < 0 || toSequence < fromSequence) {
            throw new IllegalArgumentException("Invalid replay sequence range.");
        }
        if (maxBytes < 0) throw new IllegalArgumentException("Backfill byte limit must not be negative.");
        java.util.concurrent.CompletableFuture<BackfillSelection> ready = new java.util.concurrent.CompletableFuture<>();
        Runnable select = () -> {
            try { if (ready.isCancelled()) return; ready.complete(selectBackfill(fromSequence, toSequence, maxBytes)); }
            catch (RuntimeException failure) { ready.completeExceptionally(failure); }
        };
        if (Thread.currentThread() == encoderThread) select.run();
        else enqueue(false, select);
        BackfillSelection snapshot;
        try { snapshot = ready.get(200, java.util.concurrent.TimeUnit.MILLISECONDS); }
        catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("Interrupted selecting replay backfill", interrupted);
        } catch (java.util.concurrent.TimeoutException timeout) {
            ready.cancel(false);
            throw new IllegalStateException("Replay encoder busy selecting backfill", timeout);
        } catch (java.util.concurrent.ExecutionException failure) {
            if (failure.getCause() instanceof RuntimeException runtime) throw runtime;
            throw new IllegalStateException(failure.getCause());
        }
        ReplayBuffer.Selection selected = snapshot.selected();
        byte[] active = snapshot.active();
        // Closed segments are immutable and stay valid if a concurrent flush transfers or evicts them.
        List<byte[]> groups = new ArrayList<>(selected.segments().size() + (active == null ? 0 : 1));
        for (ReplayBuffer.Segment closed : selected.segments()) groups.add(closed.records().clone());
        if (active != null) groups.add(active);
        return List.copyOf(groups);
    }

    private record BackfillSelection(ReplayBuffer.Selection selected, byte[] active) { }

    private BackfillSelection selectBackfill(long fromSequence, long toSequence, int maxBytes) {
        ReplayBuffer.Selection selected = buffer.select(fromSequence, toSequence, maxBytes);
        byte[] active = null;
        if (selected.complete() && segment != null && segmentFrames > 0
                && segmentLastSequence >= fromSequence && segmentFirstSequence <= toSequence) {
            if (ReplayBuffer.encodedSize(segment.size()) <= maxBytes - selected.bytes()) active = segment.toByteArray();
            else if (selected.segments().isEmpty()) {
                throw new IllegalArgumentException("Replay segment exceeds the backfill response limit.");
            }
        }
        return new BackfillSelection(selected, active);
    }
}
