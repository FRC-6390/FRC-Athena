package ca.frc6390.athena.mechanisms.config;

/**
 * Encoder configuration.
 *
 * <p>{@code source} is intentionally a string so it can support both integrated encoders
 * (for example {@code motor}) and external ones (for example {@code cancoder}).
 */
public record MechanismEncoderConfig(
        String source,
        Integer id,
        Integer motorId,
        Boolean absolute,
        Boolean inverted,
        Double gearRatio,
        Double conversion,
        Double conversionOffset,
        Double offset,
        Double discontinuityPoint,
        Double discontinuityRange
) {
}

