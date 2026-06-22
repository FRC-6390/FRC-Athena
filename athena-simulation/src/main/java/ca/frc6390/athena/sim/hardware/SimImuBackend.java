package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;

/**
 * Simulation IMU backend used by tests and examples.
 */
public final class SimImuBackend implements ImuBackend {
    @Override
    public boolean supports(ImuKind kind) {
        return kind.key().startsWith("sim:");
    }

    @Override
    public ImuDevice create(ImuSpec spec) {
        return new SimImuDevice(spec, 0.0);
    }
}
