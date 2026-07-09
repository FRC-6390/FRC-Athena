package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Simulation backend used by tests and examples.
 */
public final class SimMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.key().startsWith("sim:");
    }

    @Override
    public MotorHandle create(MotorDevice device) {
        return new SimMotorHandle(device);
    }
}
