package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.spec.MotorSpec;

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
     * Returns capabilities for a supported motor kind.
     *
     * @param kind motor kind
     * @return capability set
     */
    CapabilitySet capabilities(MotorKind kind);

    /**
     * Creates a runtime motor device from a validated spec.
     *
     * @param spec motor spec
     * @return runtime motor
     */
    MotorDevice create(MotorSpec spec);
}
