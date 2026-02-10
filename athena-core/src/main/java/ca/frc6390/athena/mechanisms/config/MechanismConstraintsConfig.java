package ca.frc6390.athena.mechanisms.config;

/**
 * Constraints for mechanism motion (bounds + motion limits).
 */
public record MechanismConstraintsConfig(
        Double min,
        Double max,
        Double buffer,
        MechanismMotionLimitsConfig motion
) {
}
