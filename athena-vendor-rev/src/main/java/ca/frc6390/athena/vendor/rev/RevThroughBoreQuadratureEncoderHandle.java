package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;
import edu.wpi.first.wpilibj.Encoder;
import java.util.Objects;

/** REV through-bore incremental encoder read from its A/B quadrature outputs. */
public final class RevThroughBoreQuadratureEncoderHandle implements EncoderHandle, AutoCloseable {
    private static final double ROTATIONS_PER_PULSE = 1.0 / 8192.0;

    private final EncoderDevice device;
    private final Encoder encoder;
    private boolean inputsFresh;
    private double positionRotations;
    private double velocityRotationsPerSecond;

    public RevThroughBoreQuadratureEncoderHandle(EncoderDevice device) {
        this.device = Objects.requireNonNull(device, "device");
        if (!(device.connection() instanceof HardwareAddress.Quadrature quadrature)) {
            throw new IllegalArgumentException("REV through-bore quadrature input requires A/B DIO connections.");
        }
        encoder = new Encoder(quadrature.channelA(), quadrature.channelB());
        encoder.setDistancePerPulse(ROTATIONS_PER_PULSE);
        if (quadrature.indexChannel() >= 0) {
            encoder.setIndexSource(quadrature.indexChannel(), Encoder.IndexingType.kResetOnRisingEdge);
        }
    }

    @Override
    public EncoderDevice device() { return device; }

    @Override
    public void refreshInputs() {
        positionRotations = encoder.getDistance();
        velocityRotationsPerSecond = encoder.getRate();
        inputsFresh = true;
    }

    @Override
    public double positionRotations() {
        ensureInputsFresh();
        return positionRotations;
    }

    @Override
    public double velocityRotationsPerSecond() {
        ensureInputsFresh();
        return velocityRotationsPerSecond;
    }

    @Override
    public void close() { encoder.close(); }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }
}
