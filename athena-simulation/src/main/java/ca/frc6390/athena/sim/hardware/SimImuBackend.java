package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Simulation IMU backend for real robot IMU declarations.
 */
public final class SimImuBackend implements ImuBackend {
    @Override
    public boolean supports(ImuKind kind) {
        return kind == ImuKinds.PIGEON_2
                || kind == ImuKinds.NAVX;
    }

    @Override
    public ImuHandle create(ImuDevice device) {
        return new SimImuHandle(device);
    }
}
