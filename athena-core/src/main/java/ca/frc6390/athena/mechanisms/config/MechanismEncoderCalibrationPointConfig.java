package ca.frc6390.athena.mechanisms.config;

/**
 * One calibration-map point.
 */
public record MechanismEncoderCalibrationPointConfig(
        Double input,
        Double output
) {
}
