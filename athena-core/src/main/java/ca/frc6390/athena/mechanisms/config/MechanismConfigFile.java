package ca.frc6390.athena.mechanisms.config;

/**
 * JSON/TOML-friendly mechanism configuration intended to live in deploy files.
 *
 * <p>This is intentionally data-only. Code-only concepts like hooks, lambdas, and custom control
 * loops remain in Java.
 */
public record MechanismConfigFile(
        String name,
        String mechanismType,
        String units,
        MechanismMotorsConfig motors,
        MechanismEncoderConfig encoder,
        MechanismConstraintsConfig constraints,
        MechanismSensorsConfig sensors,
        MechanismControlConfig control,
        MechanismSimConfig sim
) {
}

