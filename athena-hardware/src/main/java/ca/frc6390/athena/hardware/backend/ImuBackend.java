package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Backend contract implemented by vendor IMU adapters.
 */
public interface ImuBackend {
    /**
     * Returns whether this backend supports an IMU kind.
     *
     * @param kind IMU kind
     * @return true if supported
     */
    boolean supports(ImuKind kind);

    /**
     * Creates a runtime IMU handle from a declaration.
     *
     * @param device IMU declaration
     * @return runtime IMU
     */
    ImuHandle create(ImuDevice device);
}
