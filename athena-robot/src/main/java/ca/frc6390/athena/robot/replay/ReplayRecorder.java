package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeCycleSnapshot;
import ca.frc6390.athena.robot.RuntimeFailureReporter;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.time.LocalDateTime;
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
 * {@link #mark} and {@link #keyframe}, which may be called from any thread.</p>
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
    private Path lastFile;

    public ReplayRecorder(ReplayPolicy policy, Path directory, RuntimeFailureReporter reporter) {
        this.policy = policy;
        this.directory = directory;
        this.reporter = reporter == null ? RuntimeFailureReporter.stderr() : reporter;
        this.buffer = new ReplayBuffer(policy.bufferBytes());
    }

    /** Returns the policy in effect. */
    public ReplayPolicy policy() {
        return policy;
    }

    /** Returns the directory flushed files are written to. */
    public Path directory() {
        return directory;
    }

    /** Returns the most recent file written by this recorder. */
    public synchronized Optional<Path> lastFile() {
        return Optional.ofNullable(lastFile);
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
     * Writes everything buffered to a new file.
     *
     * @return the file, or empty when nothing was buffered or writing failed
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

    /** Closes the stream sink reference. Buffered data is not flushed. */
    public synchronized void close() {
        sink = null;
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
        metadata.put("createdAt", clock.get().toString());
        LocalDateTime now = clock.get();
        Path target = directory.resolve(ReplayFiles.fileName(match, now));
        for (int attempt = 0; attempt < 600 && java.nio.file.Files.exists(target); attempt++) {
            now = now.plusSeconds(1);
            target = directory.resolve(ReplayFiles.fileName(match, now));
        }
        try {
            ReplayFileWriter.write(target, metadata, dictionary.toByteArray(), records);
            lastFile = target;
            ReplayFiles.at(directory).enforce(policy.retention(), target.getFileName().toString());
            return Optional.of(target);
        } catch (IOException | RuntimeException exception) {
            reporter.error("Athena replay flush to " + target + " failed (" + reason + ").", exception);
            return Optional.empty();
        }
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
