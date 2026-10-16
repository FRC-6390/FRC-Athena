package ca.frc6390.athena.vision.device;

/**
 * Whether a camera's clock is trustworthy enough to use its frame timestamps as robot time.
 *
 * <p>Vision coprocessors stamp frames in robot time only after their time sync with the roboRIO
 * succeeds. This status summarizes that sync for the most recent frame the adapter read:</p>
 * <ul>
 *   <li>{@code synced} is true only when the last sync confirmation ("pong") is younger than
 *       {@value #SYNC_TIMEOUT_SECONDS} s and the frame's capture timestamp is not more than
 *       {@value #MAX_FRAME_LEAD_SECONDS} s ahead of the robot time at refresh.</li>
 *   <li>{@code secondsSinceSync} is the age of the last sync confirmation, or NaN when the camera
 *       does not report one (for example Limelight, or PhotonVision before its first pong).</li>
 * </ul>
 *
 * @param synced true when the camera clock looks synchronized to robot time
 * @param secondsSinceSync age of the last time-sync confirmation in seconds; NaN if unknown
 */
public record CameraClockStatus(boolean synced, double secondsSinceSync) {
    /** A sync confirmation older than this (seconds) means the clock is not considered synced. */
    public static final double SYNC_TIMEOUT_SECONDS = 1.0;

    /** A frame stamped this far (seconds) ahead of robot time is flagged as unsynced. */
    public static final double MAX_FRAME_LEAD_SECONDS = 0.1;

    /** Status for cameras that cannot report clock sync. */
    public static final CameraClockStatus UNKNOWN = new CameraClockStatus(false, Double.NaN);

    public CameraClockStatus {
        secondsSinceSync = Double.isFinite(secondsSinceSync) && secondsSinceSync >= 0.0
                ? secondsSinceSync
                : Double.NaN;
    }

    /**
     * Returns the status for a camera that reports no clock information.
     *
     * @return unknown status
     */
    public static CameraClockStatus unknown() {
        return UNKNOWN;
    }

    /**
     * Computes the sync status from one frame's timing.
     *
     * @param secondsSinceLastPong age of the last time-sync confirmation in seconds; NaN or negative if unknown
     * @param frameCaptureSeconds frame capture timestamp in robot time
     * @param robotTimeSeconds robot time when the adapter was refreshed
     * @return sync status
     */
    public static CameraClockStatus of(
            double secondsSinceLastPong, double frameCaptureSeconds, double robotTimeSeconds) {
        if (!Double.isFinite(secondsSinceLastPong) || secondsSinceLastPong < 0.0) {
            return UNKNOWN;
        }
        boolean pongFresh = secondsSinceLastPong < SYNC_TIMEOUT_SECONDS;
        boolean frameAhead = Double.isFinite(frameCaptureSeconds)
                && Double.isFinite(robotTimeSeconds)
                && frameCaptureSeconds - robotTimeSeconds > MAX_FRAME_LEAD_SECONDS;
        return new CameraClockStatus(pongFresh && !frameAhead, secondsSinceLastPong);
    }

    /**
     * Returns true when this status carries no clock information.
     *
     * @return true if the sync age is unknown
     */
    public boolean isUnknown() {
        return Double.isNaN(secondsSinceSync);
    }
}
