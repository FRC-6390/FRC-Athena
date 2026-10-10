package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Serves replay backfill requests from the robot's buffer. Called on the robot thread from
 * {@link NativeArcpServer#serviceRequests()}.
 */
@FunctionalInterface
public interface ArcpBackfill {

    /**
     * Returns self-contained record groups covering frame sequences {@code from..=to}. Each group is a
     * complete run of replay records (dictionary first, then a keyframe) so it decodes on its own. An empty
     * list means the range is no longer buffered.
     *
     * @param fromSequence first wanted sequence, inclusive
     * @param toSequence last wanted sequence, inclusive
     * @return record groups, oldest first
     */
    List<byte[]> records(long fromSequence, long toSequence);
}
