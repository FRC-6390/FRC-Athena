package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.mechanism.core.RecordMetadata;
import ca.frc6390.athena.mechanism.core.RuntimeMarkers;
import java.util.OptionalLong;

/**
 * {@link RuntimeMarkers} implementation backed by one {@link ReplayRecorder}. Installed by
 * {@code RobotRuntime.replay(...)} so mechanism code (persisted writes, rejected writes, staging transitions,
 * faults, interpolation records) lands in the active recording without knowing about replay.
 *
 * <p>Record metadata reports the match identity the recorder learned from the driver station (event, match type,
 * match number), the recording session as the replay id, and the latest cycle sequence as the frame index.</p>
 */
public final class ReplayRuntimeMarkers implements RuntimeMarkers {
    private final ReplayRecorder recorder;

    /**
     * Creates markers that write to a recorder.
     *
     * @param recorder recorder that receives markers and keyframe requests
     */
    public ReplayRuntimeMarkers(ReplayRecorder recorder) {
        this.recorder = java.util.Objects.requireNonNull(recorder, "recorder");
    }

    /** Returns the recorder that receives markers. */
    public ReplayRecorder recorder() {
        return recorder;
    }

    @Override
    public void mark(int kind, String message) {
        recorder.mark(kind, message == null ? "" : message);
    }

    /** Forces the next recorded cycle to be a keyframe that carries every known signal. */
    @Override
    public void requestFullCapture() {
        recorder.keyframe();
    }

    @Override
    public RecordMetadata recordMetadata() {
        ReplayMatch match = recorder.match();
        long frame = recorder.currentFrameIndex();
        return new RecordMetadata(
                recorder.currentTimestampSeconds(),
                match.matchType(),
                match.matchNumber(),
                match.eventName(),
                OptionalLong.of(recorder.sessionId()),
                frame < 0 ? OptionalLong.empty() : OptionalLong.of(frame));
    }

    /** Uses the recorder's cycle clock, so record timestamps match the marker and frame timestamps. */
    @Override
    public double clockSeconds() {
        return recorder.currentTimestampSeconds();
    }
}
