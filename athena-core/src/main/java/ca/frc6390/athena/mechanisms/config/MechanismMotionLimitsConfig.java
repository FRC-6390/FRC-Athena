package ca.frc6390.athena.mechanisms.config;

/**
 * Optional trapezoid motion limits.
 */
public record MechanismMotionLimitsConfig(
        Double maxVelocity,
        Double maxAcceleration
) {
}

