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
        segments.addLast(segment);
        bytes += segment.records().length;
        while (bytes > maxBytes && segments.size() > 1) {
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

    /** Returns copies of the segments that overlap the sequence range, oldest first. */
    List<byte[]> backfill(long fromSequence, long toSequence) {
        List<byte[]> groups = new ArrayList<>();
        for (Segment segment : segments) {
            if (segment.firstSequence() < 0) continue;
            if (segment.lastSequence() < fromSequence || segment.firstSequence() > toSequence) continue;
            groups.add(segment.records().clone());
        }
        return List.copyOf(groups);
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
