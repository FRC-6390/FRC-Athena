package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * REV motor backend boundary.
 */
public final class RevMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.key().startsWith("rev:spark-");
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
                MotorCapability.INTEGRATED_ENCODER,
                MotorCapability.ABSOLUTE_ENCODER);
    }

    @Override
    public MotorDevice create(MotorSpec spec) {
        return new RevMotorDevice(spec, spec.vendorOptions().find(RevMotorOptions.class).orElse(new RevMotorOptions()));
    }
}
