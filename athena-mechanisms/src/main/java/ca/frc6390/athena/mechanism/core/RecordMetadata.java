package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.OptionalLong;

/**
 * Identity attached to an interpolation point captured with {@code InterpolatedControlAction.record()}.
 *
 * @param timestampSeconds recording time
 * @param matchType match type, such as {@code Qualification}, or empty when unknown
 * @param matchNumber match number, or 0 when unknown
 * @param eventName event name, or empty when unknown
 * @param replayId replay file identity when a replay is being captured
 * @param frameIndex replay frame index when a replay is being captured
 */
public record RecordMetadata(
        double timestampSeconds,
        String matchType,
        int matchNumber,
        String eventName,
        OptionalLong replayId,
        OptionalLong frameIndex) {
    public RecordMetadata {
        matchType = matchType == null ? "" : matchType;
        eventName = eventName == null ? "" : eventName;
        replayId = Objects.requireNonNull(replayId, "replayId");
        frameIndex = Objects.requireNonNull(frameIndex, "frameIndex");
    }

    /** Creates metadata with only a timestamp. */
    public static RecordMetadata at(double timestampSeconds) {
        return new RecordMetadata(timestampSeconds, "", 0, "", OptionalLong.empty(), OptionalLong.empty());
    }

    /** Returns a one-line description used in markers and telemetry. */
    public String describe() {
        return "t=" + timestampSeconds
                + " matchType=" + matchType
                + " matchNumber=" + matchNumber
                + " event=" + eventName
                + " replayId=" + (replayId.isPresent() ? replayId.getAsLong() : "none")
                + " frame=" + (frameIndex.isPresent() ? frameIndex.getAsLong() : "none");
    }
}
