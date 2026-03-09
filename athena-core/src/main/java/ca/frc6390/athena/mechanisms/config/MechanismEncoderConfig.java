package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * Named encoder source configuration.
 *
 * <p>{@code source} is the encoder source kind or registry key. Supported values include:
 * {@code internal}, Athena encoder enum names such as {@code cancoder}, registry keys such as
 * {@code ctre:cancoder}, and derived kinds such as {@code crt}, {@code filter},
 * {@code differentiate}, {@code average}, {@code difference}, and {@code calibration_map}.
 *
 * <p>For hardware sources, {@code id} is the CAN/device id. For {@code internal}, {@code id}
 * refers to the owning motor id.
 */
public record MechanismEncoderConfig(
        String name,
        String source,
        Integer id,
        String canbus,
        Boolean inverted,
        Double gearRatio,
        Double conversion,
        Double offset,
        String unit,
        Double wrapsEvery,
        Double validMin,
        Double validMax,
        List<MechanismEncoderInputConfig> inputs,
        String filter,
        Double filterAlpha,
        Integer filterWindow,
        List<MechanismEncoderCalibrationPointConfig> points
) {
}
