package ca.frc6390.athena.localization.pipeline;

/**
 * Diagnostic status of one vision source (camera) as seen by a Kalman localization, recomputed every refresh.
 *
 * @param name camera name, from the camera device; a generated name for sources without one
 * @param state what the source is doing now
 * @param reason short human text naming the rejection, for example {@code ambiguity 0.31 > 0.20}; empty when OK
 * @param acceptedPerSecond frames accepted in the last second
 * @param rejectedPerSecond frames rejected in the last second, by any stage
 * @param frameAgeSeconds robot time minus the capture time of the newest frame; NaN when no frame was seen
 * @param tagCount tag count of the newest frame
 * @param secondsSinceAccepted robot time since the last accepted frame; NaN when none was accepted
 */
public record VisionSourceStatus(
        String name,
        State state,
        String reason,
        double acceptedPerSecond,
        double rejectedPerSecond,
        double frameAgeSeconds,
        int tagCount,
        double secondsSinceAccepted) {

    /**
     * Source state. Priority, highest first: UNSYNCED, STALE, OK, RECOVERING, REJECTED. NO_TARGETS is reserved:
     * a pose-only pipeline cannot tell "no targets" from "no frames", so a silent camera reports STALE.
     */
    public enum State {
        /** Newest frame is accepted. */
        OK,
        /** Newest frame was rejected by a filter or a Kalman gate. */
        REJECTED,
        /** Newest frame was held back while a rejected source proves it is self-consistent again. */
        RECOVERING,
        /** Frame timestamps are ahead of robot time: the camera clock is not synced yet. */
        UNSYNCED,
        /** No new frame for more than 0.5 s. */
        STALE,
        /** Camera connected but nothing seen. Reserved; see the class notes. */
        NO_TARGETS
    }

    public VisionSourceStatus {
        name = name == null ? "" : name;
        state = state == null ? State.STALE : state;
        reason = reason == null ? "" : reason;
    }

    /** Dashboard path of one field, for example {@code athena/vision/Intake/State}. */
    public static String path(String camera, String field) {
        return "athena/vision/" + camera + "/" + field;
    }
}
