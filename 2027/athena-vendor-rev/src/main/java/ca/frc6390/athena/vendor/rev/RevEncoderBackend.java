package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;

/**
 * REV encoder backend for standalone through-bore encoders.
 */
public final class RevEncoderBackend implements EncoderBackend {
    @Override
    public boolean supports(EncoderKind kind) {
        return kind == AthenaEncoder.REV_THROUGH_BORE || kind.key().equals("rev:through-bore");
    }

    @Override
    public EncoderDevice create(EncoderSpec spec) {
        return new RevThroughBoreEncoderDevice(spec);
    }
}
