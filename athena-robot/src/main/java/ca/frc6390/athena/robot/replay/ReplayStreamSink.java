package ca.frc6390.athena.robot.replay;

import java.util.List;

/**
 * Destination for live replay records, used by the ARCP stream once it exists. Methods run on the ordered replay worker and must not block it indefinitely.
 */
public interface ReplayStreamSink {
    /** Leaves room for the control reply envelope inside the protocol's 1 MiB message limit. */
    int DEFAULT_BACKFILL_BYTES = (1 << 20) - 64;

    /** Publishes one or more complete DICTIONARY records. */
    void publishDictionary(byte[] records);

    /** Publishes the complete FRAME record for one sequence number. */
    void publishFrame(long sequence, byte[] records);

    /** Publishes one MARKER record, tagged with the latest frame sequence. */
    void publishMarker(long sequence, byte[] record);

    /**
     * Receives a backfill handle once, when the sink is installed. A client that detects a sequence gap calls
     * {@link ReplayBackfill#records(long, long)} to fetch self-contained record groups from the buffer.
     *
     * @param backfill backfill handle
     */
    default void attach(ReplayBackfill backfill) {
    }

    /** Sequence-number backfill from the in-RAM buffer, safe to call on a backfill worker. */
    interface ReplayBackfill {
        /**
         * Returns buffered record groups covering the requested sequence range. Each group starts with a
         * dictionary and a keyframe, so it decodes on its own. Groups may include frames before
         * {@code fromSequence}. The built-in provider returns a bounded oldest-first window using
         * {@link ReplayStreamSink#DEFAULT_BACKFILL_BYTES}; callers can request a smaller limit with the overload.
         *
         * @param fromSequence first wanted sequence, inclusive
         * @param toSequence last wanted sequence, inclusive
         * @return record groups in order; empty when the range is no longer buffered
         */
        List<byte[]> records(long fromSequence, long toSequence);

        /**
         * Returns an oldest-first window bounded by encoded bytes, including each group's length prefix.
         * Large-history providers should override this overload to select before copying history.
         */
        default List<byte[]> records(long fromSequence, long toSequence, int maxBytes) {
            if (maxBytes < 0) throw new IllegalArgumentException("Backfill byte limit must not be negative.");
            List<byte[]> bounded = new java.util.ArrayList<>();
            int used = 0;
            for (byte[] group : records(fromSequence, toSequence)) {
                long size = ReplayBuffer.encodedSize(group.length);
                if (size > maxBytes - used) {
                    if (bounded.isEmpty()) {
                        throw new IllegalArgumentException("Replay segment exceeds the backfill response limit.");
                    }
                    break;
                }
                bounded.add(group);
                used += (int) size;
            }
            return List.copyOf(bounded);
        }

    }
}
