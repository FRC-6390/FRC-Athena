package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;

/**
 * Simulation encoder backend for real robot encoder declarations.
 */
public final class SimEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == EncoderKinds.CANCODER
                || kind == EncoderKinds.REV_THROUGH_BORE
                || kind == EncoderKinds.INTEGRATED_MOTOR;
    }

    @Override
    public EncoderHandle create(EncoderDevice device) {
        return new SimEncoderHandle(device);
    }
}
