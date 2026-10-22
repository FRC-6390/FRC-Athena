package ca.frc6390.athena.robot.replay;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Bounded in-RAM ring of closed replay segments. Each segment starts with a full dictionary and a keyframe,
 * so any retained segment can be written, streamed, or backfilled on its own. The oldest segments are
 * evicted first; the newest segment is always kept.
 */
final class ReplayBuffer {
    /**
     * One closed segment.
     *
     * @param records encoded DICTIONARY, FRAME and MARKER records
     * @param firstSequence first frame sequence, or -1 when the segment has no frames
     * @param lastSequence last frame sequence, or -1
     * @param firstTimestampSeconds first timestamp
     * @param lastTimestampSeconds last timestamp
     */
    record Segment(byte[] records, long firstSequence, long lastSequence,
                   double firstTimestampSeconds, double lastTimestampSeconds) {
    }

    private final long maxBytes;
    private final ArrayDeque<Segment> segments = new ArrayDeque<>();
    private long bytes;
    private long evicted;

    ReplayBuffer(long maxBytes) {
        this.maxBytes = Math.max(0, maxBytes);
    }

    void add(Segment segment) {
        add(segment, maxBytes);
    }

    /** A slow disk writer shares the history budget with this ring; always retain the newest segment. */
    void add(Segment segment, long availableBytes) {
        segments.addLast(segment);
        bytes += segment.records().length;
        long limit = Math.min(maxBytes, Math.max(0, availableBytes));
        while (bytes > limit && segments.size() > 1) {
            bytes -= segments.removeFirst().records().length;
            evicted++;
        }
    }

    /** Removes and returns all segments, oldest first. */
    List<Segment> drain() {
        List<Segment> drained = new ArrayList<>(segments);
        segments.clear();
        bytes = 0;
        return List.copyOf(drained);
    }

    /** Selected immutable segment references; byte accounting includes each group's length prefix. */
    record Selection(List<Segment> segments, int bytes, boolean complete) {
    }

    Selection select(long fromSequence, long toSequence, int maxBytes) {
        if (maxBytes < 0) throw new IllegalArgumentException("Backfill byte limit must not be negative.");
        List<Segment> selected = new ArrayList<>();
        int used = 0;
        for (Segment segment : segments) {
            if (segment.firstSequence() < 0) continue;
            if (segment.lastSequence() < fromSequence || segment.firstSequence() > toSequence) continue;
            long size = encodedSize(segment.records().length);
            if (size > maxBytes - used) {
                if (selected.isEmpty()) {
                    throw new IllegalArgumentException("Replay segment exceeds the backfill response limit.");
                }
                return new Selection(List.copyOf(selected), used, false);
            }
            selected.add(segment);
            used += (int) size;
        }
        return new Selection(List.copyOf(selected), used, true);
    }

    /** Length-prefixed byte size on the control wire, computed without copying the segment. */
    static long encodedSize(int length) {
        int prefix = 1;
        for (int rest = length; (rest >>>= 7) != 0;) prefix++;
        return (long) length + prefix;
    }

    /** Returns bounded copies, oldest first. Selection happens before any segment is cloned. */
    List<byte[]> backfill(long fromSequence, long toSequence, int maxBytes) {
        Selection selection = select(fromSequence, toSequence, maxBytes);
        List<byte[]> groups = new ArrayList<>(selection.segments().size());
        for (Segment segment : selection.segments()) groups.add(segment.records().clone());
        return List.copyOf(groups);
    }

    List<byte[]> backfill(long fromSequence, long toSequence) {
        return backfill(fromSequence, toSequence, ReplayStreamSink.DEFAULT_BACKFILL_BYTES);
    }

    boolean isEmpty() {
        return segments.isEmpty();
    }

    long bytes() {
        return bytes;
    }

    long evictedSegments() {
        return evicted;
    }
}
