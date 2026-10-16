package ca.frc6390.athena.localization.pipeline;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * The latest vision source statuses of every Kalman localization that has refreshed in this process. The ARCP
 * bridge reads it to publish {@code athena/vision/<camera>/...} each cycle, so the diagnostics reach the dashboard
 * without the robot wiring them up. When two nodes report the same camera name, one of their statuses is shown.
 */
public final class VisionStatusRegistry {
    private static final Map<Object, List<VisionSourceStatus>> BY_OWNER = new WeakHashMap<>();

    private VisionStatusRegistry() {
    }

    static void publish(Object owner, List<VisionSourceStatus> statuses) {
        synchronized (BY_OWNER) {
            BY_OWNER.put(owner, List.copyOf(statuses));
        }
    }

    /**
     * Returns the statuses of all refreshed Kalman localizations, one per camera name, in registration order.
     *
     * @return statuses; empty before any Kalman localization has refreshed
     */
    public static List<VisionSourceStatus> latest() {
        Map<String, VisionSourceStatus> byName = new LinkedHashMap<>();
        synchronized (BY_OWNER) {
            for (List<VisionSourceStatus> statuses : BY_OWNER.values()) {
                for (VisionSourceStatus status : statuses) {
                    byName.put(status.name(), status);
                }
            }
        }
        return new ArrayList<>(byName.values());
    }
}
