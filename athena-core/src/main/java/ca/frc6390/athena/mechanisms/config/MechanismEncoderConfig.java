package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * Named encoder source configuration.
 *
 * <p>{@code source} is the encoder source kind or registry key. Supported values include:
 * {@code internal}, Athena encoder enum names such as {@code cancoder}, registry keys such as
 * {@code ctre:cancoder}, and {@code crt} for derived Chinese Remainder sources.
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
        List<MechanismEncoderCrtInputConfig> crtInputs
) {
}
