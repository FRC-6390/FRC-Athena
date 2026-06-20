package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;

/**
 * CTRE encoder backend for CANcoder devices.
 */
public final class CtreEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == AthenaEncoder.CANCODER || kind.key().equals("ctre:cancoder");
    }

    @Override
    public EncoderDevice create(EncoderSpec spec) {
        return new CtreEncoderDevice(spec);
    }
}
