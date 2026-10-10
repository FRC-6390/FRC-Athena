package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Snapshot of the server state.
 *
 * @param sessions connected control sessions
 * @param subscriptions active subscriptions across sessions
 * @param bytesLastWindow realtime bytes sent in the last budget window
 * @param throttled subscriptions whose rate the bandwidth budget lowered, one description per entry
 */
public record ArcpStatus(long sessions, long subscriptions, long bytesLastWindow, List<String> throttled) {

    /** Status reported when no server is running. */
    public static final ArcpStatus STOPPED = new ArcpStatus(0, 0, 0, List.of());

    public ArcpStatus {
        throttled = List.copyOf(throttled);
    }
}
