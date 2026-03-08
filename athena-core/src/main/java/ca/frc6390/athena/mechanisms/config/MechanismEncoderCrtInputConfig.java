package ca.frc6390.athena.mechanisms.config;

/**
 * One CRT input reference.
 */
public record MechanismEncoderCrtInputConfig(
        String source,
        Integer modulus
) {
}
