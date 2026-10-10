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
import java.util.HashSet;
import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
 */
public final class ReplayRecorder {
    /** Segments close at a cycle boundary once they reach this size. */
    static final int SEGMENT_TARGET_BYTES = 64 * 1024;

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
    private int segmentFrames;
    private long segmentFirstSequence = -1;
    private long segmentLastSequence = -1;
    private double segmentFirstTimestamp;
    private double segmentLastTimestamp;
    private ReplayMatch match = ReplayMatch.none();
    private boolean enabledSinceFlush;
    private long lastSequence = -1;
    private double lastTimestampSeconds;
    private ReplayStreamSink sink;
    private final Set<String> issuedNames = new HashSet<>();
    private ReplayWriter writer;
    private final long sessionId = SESSIONS.incrementAndGet();

    /** Session identities are unique within a JVM and increase over time (seeded from the wall clock). */
    private static final java.util.concurrent.atomic.AtomicLong SESSIONS =
            new java.util.concurrent.atomic.AtomicLong(System.currentTimeMillis() / 1000L * 1000L);

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
    public synchronized long currentFrameIndex() {
        return lastSequence;
    }

    /** Returns the timestamp of the latest recorded cycle, in seconds. */
    public synchronized double currentTimestampSeconds() {
        return lastTimestampSeconds;
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
        ReplayWriter current = writer;
        return current == null || current.awaitIdle(timeout);
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

    /** Returns the match identity recorded in new files. */
    public synchronized ReplayMatch match() {
        return match;
    }

    /**
     * Installs a stream sink. Streaming happens only when the policy enables it.
     *
     * @param sink sink, or null to remove it
     */
    public synchronized void streamSink(ReplayStreamSink sink) {
        this.sink = sink;
        if (sink != null) {
            sink.attach(this::backfill);
        }
    }

    /** Returns true when frames are being published to a sink. */
    public synchronized boolean streaming() {
        return policy.streamEnabled() && sink != null;
    }

    /**
     * Records one cycle.
     *
     * @param snapshot runtime cycle snapshot
     * @param mechanismInputs hardware inputs the mechanisms used, or null to use the snapshot's
     * @param outputs resolved mechanism outputs, or null for cycles that ran no mechanisms
     */
    public synchronized void record(
            RuntimeCycleSnapshot snapshot,
            HardwareCycleSnapshot mechanismInputs,
            List<ResolvedOutput> outputs) {
        if (segment == null || segment.size() >= SEGMENT_TARGET_BYTES) {
            closeSegment();
            openSegment();
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
        if (streaming()) {
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
    public synchronized void mark(int kind, String message) {
        mark(kind, message, 0);
    }

    /**
     * Adds a marker that refers to a signal.
     *
     * @param kind marker kind
     * @param message marker message
     * @param signalId signal id, or 0 for none
     */
    public synchronized void mark(int kind, String message, int signalId) {
        if (segment == null) {
            openSegment();
        }
        markerScratch.clear();
        encoder.writeMarker(markerScratch, lastTimestampSeconds, kind, message, signalId);
        segment.bytes(markerScratch.buffer(), 0, markerScratch.size());
        if (streaming()) {
            sink.publishMarker(Math.max(0, lastSequence), markerScratch.toByteArray());
        }
    }

    /** Forces the next recorded cycle to be a keyframe with every known signal. */
    public synchronized void keyframe() {
        encoder.requestKeyframe();
    }

    /**
     * Hands everything buffered to the writer thread, which writes it to a new file. Robot-thread cost is O(number
     * of buffered segments) reference moves plus encoding of the dictionary; no compression or I/O happens here.
     * The returned path is where the file will be written. It is not guaranteed to exist until
     * {@link #awaitFlushed(Duration)} returns, and the writer may choose a later name if a file appeared there.
     * Use {@link #lastFile()} for the name actually written.
     *
     * @return the destination, or empty when nothing was buffered or the recorder is closed
     */
    public synchronized Optional<Path> flush() {
        return flush("explicit");
    }

    /** Called when the FMS or driver station identifies a match. */
    public synchronized void matchIdentified(ReplayMatch identified) {
        if (identified == null || !identified.identified() || identified.equals(match)) {
            return;
        }
        flush("match-identified");
        match = identified;
        mark(ReplayMarkers.MATCH_IDENTIFIED, identified.eventName() + " " + identified.typeCode()
                + identified.matchNumber());
    }

    /** Called when autonomous or teleop starts. */
    public synchronized void enabledPeriod(int markerKind, String message) {
        enabledSinceFlush = true;
        mark(markerKind, message);
    }

    /** Called when teleop ends during a match. */
    public synchronized void matchEnd() {
        mark(ReplayMarkers.MATCH_END, "teleop ended");
        enabledSinceFlush = false;
        if (policy.flushes(ReplayPolicy.Flush.MATCH_END)) {
            flush("match-end");
        }
    }

    /** Called when the robot is disabled. */
    public synchronized void disabled() {
        mark(ReplayMarkers.DISABLE, "disabled");
        boolean afterMatch = enabledSinceFlush;
        enabledSinceFlush = false;
        if (afterMatch && policy.flushes(ReplayPolicy.Flush.DISABLE_AFTER_MATCH)) {
            flush("disable-after-match");
        }
    }

    /** Called when battery voltage recovers from a brownout. */
    public synchronized void brownoutRecovered() {
        mark(ReplayMarkers.BROWNOUT, "brownout recovered");
        if (policy.flushes(ReplayPolicy.Flush.BROWNOUT_RECOVERY)) {
            flush("brownout-recovery");
        }
    }

    /**
     * Closes the stream sink reference and stops the writer thread once queued flushes are written. Buffered data
     * that was not flushed is discarded.
     */
    public synchronized void close() {
        sink = null;
        if (writer != null) {
            writer.close();
        }
    }

    void clock(Supplier<LocalDateTime> clock) {
        this.clock = clock;
    }

    private synchronized Optional<Path> flush(String reason) {
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
        metadata.put("event", match.eventName());
        metadata.put("matchType", match.matchType());
        metadata.put("matchNumber", String.valueOf(match.matchNumber()));
        metadata.put("identified", String.valueOf(match.identified()));
        metadata.put("firstSequence", String.valueOf(segments.get(0).firstSequence()));
        metadata.put("lastSequence", String.valueOf(segments.get(segments.size() - 1).lastSequence()));
        metadata.put("firstTimestampSeconds", String.valueOf(segments.get(0).firstTimestampSeconds()));
        metadata.put("lastTimestampSeconds",
                String.valueOf(segments.get(segments.size() - 1).lastTimestampSeconds()));
        LocalDateTime now = clock.get();
        metadata.put("createdAt", now.toString());
        String name = ReplayFiles.fileName(match, now);
        // Names issued by this recorder are unique within the second; the writer checks the disk for the rest.
        for (int attempt = 0; attempt < 600 && issuedNames.contains(name); attempt++) {
            now = now.plusSeconds(1);
            name = ReplayFiles.fileName(match, now);
        }
        issuedNames.add(name);
        Path target = directory.resolve(name);
        if (!replayWriter().submit(new ReplayWriter.Job(target, metadata, dictionary.toByteArray(), records, reason))) {
            reporter.error("Athena replay flush to " + target + " was refused; the recorder is closed.", null);
            return Optional.empty();
        }
        return Optional.of(target);
    }

    private ReplayWriter replayWriter() {
        if (writer == null) {
            writer = new ReplayWriter(reporter, protectedName -> ReplayFiles.at(directory)
                    .enforce(policy.retention(), protectedName));
        }
        return writer;
    }

    private void openSegment() {
        segment = new WireWriter(SEGMENT_TARGET_BYTES + 8192);
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
            buffer.add(new ReplayBuffer.Segment(
                    segment.toByteArray(),
                    segmentFirstSequence,
                    segmentLastSequence,
                    segmentFirstTimestamp,
                    segmentLastTimestamp));
        }
        segment = null;
    }

    /** Returns buffered and open-segment groups covering a sequence range, oldest first. */
    synchronized List<byte[]> backfill(long fromSequence, long toSequence) {
        List<byte[]> groups = new ArrayList<>(buffer.backfill(fromSequence, toSequence));
        if (segment != null && segmentFrames > 0 && segmentLastSequence >= fromSequence
                && segmentFirstSequence <= toSequence) {
            groups.add(segment.toByteArray());
        }
        return List.copyOf(groups);
    }
}
