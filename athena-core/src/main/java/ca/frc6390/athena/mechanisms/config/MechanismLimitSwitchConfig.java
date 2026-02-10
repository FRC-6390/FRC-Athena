package ca.frc6390.athena.mechanisms.config;

/**
 * Generic limit switch config.
 *
 * <p>Maps cleanly to Athena's {@code GenericLimitSwitchConfig}.
 */
public record MechanismLimitSwitchConfig(
        Integer id,
        Boolean inverted,
        Double position,
        Boolean hardstop,
        String blockDirection,
        String name,
        Double delaySeconds
) {
}

