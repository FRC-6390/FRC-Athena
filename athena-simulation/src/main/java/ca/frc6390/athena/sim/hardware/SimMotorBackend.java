package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Simulation motor backend for real robot motor declarations.
 */
public final class SimMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.motorKind() instanceof MotorKinds;
    }

    @Override
    public MotorHandle create(MotorDevice device) {
        return new SimMotorHandle(device);
    }
}
