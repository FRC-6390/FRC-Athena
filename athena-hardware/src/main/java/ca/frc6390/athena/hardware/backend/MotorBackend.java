package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Backend contract implemented by vendor motor adapters.
 */
public interface MotorBackend {
    /**
     * Returns whether this backend can create the provided kind.
     *
     * @param kind motor kind
     * @return true if supported
     */
    boolean supports(MotorKind kind);

    /**
     * Creates a runtime motor handle from a declaration.
     *
     * @param device motor declaration
     * @return runtime motor
     */
    MotorHandle create(MotorDevice device);
}
