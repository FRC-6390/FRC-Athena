package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;

/**
 * Backend contract implemented by vendor encoder adapters.
 */
public interface EncoderBackend {
    /**
     * Returns whether this backend supports an encoder kind.
     *
     * @param kind encoder kind
     * @return true if supported
     */
    boolean supports(EncoderKind kind);

    /**
     * Creates a runtime encoder from a validated spec.
     *
     * @param spec encoder spec
     * @return runtime encoder
     */
    EncoderDevice create(EncoderSpec spec);
}
