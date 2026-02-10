package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * External sensors attached to a mechanism (limit switches, beam breaks, etc).
 */
public record MechanismSensorsConfig(
        List<MechanismLimitSwitchConfig> limitSwitches
) {
}

