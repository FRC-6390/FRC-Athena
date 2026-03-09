package ca.frc6390.athena.mechanisms.config;

/**
 * One derived encoder input reference.
 */
public record MechanismEncoderInputConfig(
        String source,
        String signal,
        Integer modulus
) {
}
