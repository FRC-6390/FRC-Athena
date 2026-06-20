package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * CTRE motor backend boundary.
 */
public final class CtreMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind == AthenaMotor.TALON_FX
                || kind == AthenaMotor.KRAKEN_X60
                || kind == AthenaMotor.KRAKEN_X44
                || kind.key().equals("ctre:talon-fx")
                || kind.key().equals("ctre:kraken-x60")
                || kind.key().equals("ctre:kraken-x44");
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
        return new CtreMotorDevice(spec, spec.vendorOptions().find(CtreMotorOptions.class).orElse(new CtreMotorOptions()));
    }
}
