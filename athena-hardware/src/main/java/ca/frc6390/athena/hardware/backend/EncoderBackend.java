package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.device.EncoderDevice;

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
     * Creates a runtime encoder handle from a declaration.
     *
     * @param device encoder declaration
     * @return runtime encoder
     */
    EncoderHandle create(EncoderDevice device);
}
