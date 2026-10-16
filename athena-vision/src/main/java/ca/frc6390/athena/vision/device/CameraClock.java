package ca.frc6390.athena.vision.device;

/**
 * Camera-side clock health, implemented by camera declarations and adapters that can observe
 * time sync with the robot. Cameras that cannot report sync keep the default {@link CameraClockStatus#unknown()}.
 *
 * <p>Read {@link #clockStatus()} to publish a per-camera value, for example telemetry key
 * {@code athena/vision/<camera>/TimeSync} with {@code clockStatus().synced()}.</p>
 */
public interface CameraClock {
    /**
     * Returns the clock status as of the most recent camera refresh.
     *
     * @return clock status; never null
     */
    default CameraClockStatus clockStatus() {
        return CameraClockStatus.unknown();
    }
}
