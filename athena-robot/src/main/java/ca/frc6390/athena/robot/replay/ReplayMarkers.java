package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.runtime.wire.Wire;

/**
 * Entry point for other modules to add markers and request keyframes in the active replay recorder.
 *
 * <p>Marker kinds 1 to 14 are the appendix E1-W kinds. Kind {@link #BROWNOUT} (14, {@code Wire.MARKER_BROWNOUT})
 * is the brownout-recovered marker. Application kinds should start at {@link #USER_BASE} plus one.</p>
 */
public final class ReplayMarkers {
    public static final int BOOT = Wire.MARKER_BOOT;
    public static final int MATCH_IDENTIFIED = Wire.MARKER_MATCH_IDENTIFIED;
    public static final int AUTO_START = Wire.MARKER_AUTO_START;
    public static final int TELEOP_START = Wire.MARKER_TELEOP_START;
    public static final int MATCH_END = Wire.MARKER_MATCH_END;
    public static final int DISABLE = Wire.MARKER_DISABLE;
    public static final int FAULT = Wire.MARKER_FAULT;
    public static final int STAGE_CHANGE = Wire.MARKER_STAGE_CHANGE;
    public static final int CHECK = Wire.MARKER_CHECK;
    public static final int OVERRIDE = Wire.MARKER_OVERRIDE;
    public static final int PERSISTED_WRITE = Wire.MARKER_PERSISTED_WRITE;
    public static final int RECORD = Wire.MARKER_RECORD;
    public static final int TEST_START = Wire.MARKER_TEST_START;
    /** Brownout recovered (appendix E1-W kind 14). */
    public static final int BROWNOUT = Wire.MARKER_BROWNOUT;
    public static final int USER_BASE = Wire.MARKER_USER_BASE;

    private static volatile ReplayRecorder active;

    private ReplayMarkers() {
    }

    /**
     * Adds a marker to the active recorder, if a robot runtime has replay enabled. Safe from any thread.
     *
     * @param kind marker kind
     * @param message marker message
     */
    public static void mark(int kind, String message) {
        ReplayRecorder recorder = active;
        if (recorder != null) {
            recorder.mark(kind, message);
        }
    }

    /**
     * Forces a full-detail keyframe in the active recorder. Used before an A4 record() capture.
     */
    public static void keyframe() {
        ReplayRecorder recorder = active;
        if (recorder != null) {
            recorder.keyframe();
        }
    }

    /** Makes a recorder the target of {@link #mark} and {@link #keyframe}. Called by the runtime. */
    public static void activate(ReplayRecorder recorder) {
        active = recorder;
    }
}
