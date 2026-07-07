package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.control.RobotVelocity;

/**
 * Robot velocity measurement.
 *
 * @param speeds measured speeds
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
public record VelocityMeasurement(
        RobotVelocity speeds,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public VelocityMeasurement {
        speeds = speeds == null ? RobotVelocity.zero() : speeds;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        latencySeconds = Double.isFinite(latencySeconds) ? Math.max(0.0, latencySeconds) : 0.0;
    }
}
