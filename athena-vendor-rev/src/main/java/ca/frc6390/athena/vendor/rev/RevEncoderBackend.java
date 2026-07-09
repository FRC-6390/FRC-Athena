package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwarePort;

/**
 * REV encoder backend for standalone through-bore encoders.
 */
public final class RevEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == EncoderKinds.REV_THROUGH_BORE || kind.key().equals("rev:through-bore");
    }

    @Override
    public boolean supports(EncoderDevice device) {
        return supports(device.kind()) && device.port() instanceof HardwarePort.Dio;
    }

    @Override
    public EncoderHandle create(EncoderDevice device) {
        return new RevThroughBoreEncoderHandle(device);
    }
}
