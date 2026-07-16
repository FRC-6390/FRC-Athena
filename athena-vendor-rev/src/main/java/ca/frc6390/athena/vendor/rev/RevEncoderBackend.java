package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;

/**
 * REV encoder backend for standalone through-bore encoders.
 */
public final class RevEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == EncoderKinds.REV_THROUGH_BORE
                || kind == EncoderKinds.REV_THROUGH_BORE_V2
                || kind == EncoderKinds.REV_THROUGH_BORE_QUADRATURE
                || kind.key().startsWith("rev:through-bore");
    }

    @Override
    public boolean supports(EncoderDevice device) {
        if (!supports(device.kind())) {
            return false;
        }
        return isQuadrature(device.kind())
                ? device.connection() instanceof HardwareAddress.Quadrature
                : device.connection() instanceof HardwareAddress.Dio;
    }

    @Override
    public EncoderHandle create(EncoderDevice device) {
        return isQuadrature(device.kind())
                ? new RevThroughBoreQuadratureEncoderHandle(device)
                : new RevThroughBoreEncoderHandle(device);
    }

    private static boolean isQuadrature(EncoderKind kind) {
        return kind == EncoderKinds.REV_THROUGH_BORE_QUADRATURE
                || kind.key().equals("rev:through-bore-quadrature");
    }
}
