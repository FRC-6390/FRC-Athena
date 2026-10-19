package ca.frc6390.athena.vision.device;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

/**
 * Whether a camera's clock is trustworthy enough to use its frame timestamps as robot time.
 *
 * <p>Vision coprocessors stamp frames in robot time only after their time sync with the roboRIO
 * succeeds. This status summarizes that sync for the most recent frame the adapter read:</p>
 * <ul>
 *   <li>{@code synced} is true only when the last sync confirmation ("pong") is younger than
 *       {@link #SYNC_TIMEOUT} and the frame's capture timestamp is not more than
 *       {@link #MAX_FRAME_LEAD} ahead of the robot time at refresh.</li>
 *   <li>{@code timeSinceSync} is the age of the last sync confirmation, or NaN when the camera
 *       does not report one (for example Limelight, or PhotonVision before its first pong).</li>
 * </ul>
 *
 * @param synced true when the camera clock looks synchronized to robot time
 * @param timeSinceSync age of the last time-sync confirmation; NaN seconds if unknown
 */
public record CameraClockStatus(boolean synced, Time timeSinceSync) {
    /** A sync confirmation older than this means the clock is not considered synced. */
    public static final Time SYNC_TIMEOUT = Seconds.of(1.0);

    /** A frame stamped this far ahead of robot time is flagged as unsynced. */
    public static final Time MAX_FRAME_LEAD = Seconds.of(0.1);

    /** Status for cameras that cannot report clock sync. */
    public static final CameraClockStatus UNKNOWN = new CameraClockStatus(false, Seconds.of(Double.NaN));

    public CameraClockStatus {
        double age = timeSinceSync == null ? Double.NaN : timeSinceSync.in(Seconds);
        timeSinceSync = Double.isFinite(age) && age >= 0.0 ? timeSinceSync : Seconds.of(Double.NaN);
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
        boolean pongFresh = secondsSinceLastPong < SYNC_TIMEOUT.in(Seconds);
        boolean frameAhead = Double.isFinite(frameCaptureSeconds)
                && Double.isFinite(robotTimeSeconds)
                && frameCaptureSeconds - robotTimeSeconds > MAX_FRAME_LEAD.in(Seconds);
        return new CameraClockStatus(pongFresh && !frameAhead, Seconds.of(secondsSinceLastPong));
    }

    /**
     * Returns true when this status carries no clock information.
     *
     * @return true if the sync age is unknown
     */
    public boolean isUnknown() {
        return Double.isNaN(timeSinceSync.in(Seconds));
    }
}
