package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Serves replay backfill requests on a bounded background worker. Implementations must be thread-safe;
 * neither the robot thread nor its hardware access scope is available here.
 */
@FunctionalInterface
public interface ArcpBackfill {

    /**
     * Returns self-contained record groups covering frame sequences {@code from..=to}. Each group is a
     * complete run of replay records (dictionary first, then a keyframe) so it decodes on its own. An empty
     * list means the range is no longer buffered. A bounded response may cover an oldest-first prefix of a
     * large requested range; its frame sequences identify the next window to request.
     *
     * @param fromSequence first wanted sequence, inclusive
     * @param toSequence last wanted sequence, inclusive
     * @return record groups, oldest first
     */
    List<byte[]> records(long fromSequence, long toSequence);

    /**
     * Returns a window bounded by encoded group bytes (including length prefixes). History providers should
     * override this overload to apply the budget before cloning or materializing their records.
     * The default preserves compatibility with small, existing providers.
     */
    default List<byte[]> records(long fromSequence, long toSequence, int maxBytes) {
        return bounded(records(fromSequence, toSequence), maxBytes);
    }

    static List<byte[]> bounded(List<byte[]> groups, int maxBytes) {
        if (maxBytes < 0) throw new IllegalArgumentException("Backfill byte limit must not be negative.");
        List<byte[]> bounded = new java.util.ArrayList<>();
        int used = 0;
        for (byte[] group : groups) {
            long size = (long) group.length + prefixSize(group.length);
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

    private static int prefixSize(int length) {
        int size = 1;
        for (int rest = length; (rest >>>= 7) != 0;) size++;
        return size;
    }

}
