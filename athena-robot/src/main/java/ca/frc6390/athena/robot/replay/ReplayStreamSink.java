package ca.frc6390.athena.robot.replay;

import java.util.List;

/**
 * Destination for live replay records, used by the ARCP stream once it exists. Methods run on the robot
 * thread and must not block.
 */
public interface ReplayStreamSink {
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

    /** Sequence-number backfill from the in-RAM buffer. */
    interface ReplayBackfill {
        /**
         * Returns buffered record groups covering the requested sequence range. Each group starts with a
         * dictionary and a keyframe, so it decodes on its own. Groups may include frames before
         * {@code fromSequence}.
         *
         * @param fromSequence first wanted sequence, inclusive
         * @param toSequence last wanted sequence, inclusive
         * @return record groups in order; empty when the range is no longer buffered
         */
        List<byte[]> records(long fromSequence, long toSequence);
    }
}
