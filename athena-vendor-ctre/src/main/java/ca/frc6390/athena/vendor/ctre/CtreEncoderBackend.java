package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwarePort;

/**
 * CTRE encoder backend for CANcoder devices.
 */
public final class CtreEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == EncoderKinds.CANCODER || kind.key().equals("ctre:cancoder");
    }

    @Override
    public boolean supports(EncoderDevice device) {
        return supports(device.kind()) && device.port() instanceof HardwarePort.Can;
    }

    @Override
    public EncoderHandle create(EncoderDevice device) {
        return new CtreEncoderHandle(device);
    }
}
