package ca.frc6390.athena.hardware.examples;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.VirtualImu;

/**
 * Examples for edge-condition tuning on IMU and encoder wrappers.
 */
public final class ImuEncoderEdgeExamples {
    private ImuEncoderEdgeExamples() {}

    public static VirtualImu createVirtualImu(Imu delegate) {
        return new VirtualImu(delegate);
    }

    public static void configureMaxSpeedWindow(VirtualImu imu, double windowSeconds) {
        imu.setMaxSpeedWindowSeconds(windowSeconds);
    }

    public static void configureEncoderDiscontinuity(Encoder encoder, double point, double range) {
        encoder.setDiscontinuityPoint(point);
        encoder.setDiscontinuityRange(range);
    }
}
