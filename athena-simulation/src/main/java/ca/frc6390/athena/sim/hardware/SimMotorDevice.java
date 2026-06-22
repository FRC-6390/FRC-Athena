package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * In-memory simulation motor device.
 *
 * @param spec normalized motor spec
 */
public record SimMotorDevice(MotorSpec spec) implements MotorDevice {
    @Override
    public void setPercentOutput(double percent) {
        // Simulation stateful motor models own rich state; this boundary keeps direct
        // backend-created devices commandable for runtime adapter tests.
    }

    @Override
    public void setVoltage(double volts) {
        setPercentOutput(volts / 12.0);
    }
}
