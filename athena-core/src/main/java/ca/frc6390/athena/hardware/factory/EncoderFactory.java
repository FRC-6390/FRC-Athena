package ca.frc6390.athena.hardware.factory;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;

/**
 * Service provider interface for creating encoders from a {@link EncoderConfig}.
 */
public interface EncoderFactory {
    boolean supports(EncoderType type);

    Encoder create(EncoderConfig config);
}
