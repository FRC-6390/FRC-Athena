package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * Motor group configuration.
 *
 * <p>Top-level fields act as defaults for each controller entry.
 */
public record MechanismMotorsConfig(
        String canbus,
        String neutralMode,
        Double currentLimit,
        List<MechanismMotorConfig> controllers
) {
}

