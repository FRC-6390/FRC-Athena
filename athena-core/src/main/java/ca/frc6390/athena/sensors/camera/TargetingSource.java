package ca.frc6390.athena.sensors.camera;

import java.util.OptionalDouble;

/**
 * Camera interface for providers that supply target yaw/pitch/range data.
 */
public interface TargetingSource {
    boolean hasValidTarget();

    OptionalDouble getTargetYawDegrees();

    OptionalDouble getTargetPitchDegrees();

    OptionalDouble getTargetDistanceMeters();
}
