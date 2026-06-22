package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * Simulation backend used by tests and examples.
 */
public final class SimMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.key().startsWith("sim:");
    }

    @Override
    public CapabilitySet capabilities(MotorKind kind) {
        return CapabilitySet.of(
                MotorCapability.PERCENT_OUTPUT,
                MotorCapability.VOLTAGE_OUTPUT,
                MotorCapability.POSITION_CLOSED_LOOP,
                MotorCapability.VELOCITY_CLOSED_LOOP,
                MotorCapability.CURRENT_LIMIT,
                MotorCapability.NEUTRAL_MODE,
                MotorCapability.INTEGRATED_ENCODER);
    }

    @Override
    public MotorDevice create(MotorSpec spec) {
        return new SimMotorDevice(spec);
    }
}
