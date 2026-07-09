package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import java.util.Objects;

final class SimEncoder implements EncoderHandle {
    private final EncoderHandle delegate;
    private double position;
    private double velocity;

    SimEncoder(EncoderHandle delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.position = delegate.positionRotations();
        this.velocity = delegate.velocityRotationsPerSecond();
    }

    @Override
    public EncoderDevice device() {
        return delegate.device();
    }

    @Override
    public double positionRotations() {
        return position;
    }

    @Override
    public double absolutePositionRotations() {
        return position;
    }

    @Override
    public double velocityRotationsPerSecond() {
        return velocity;
    }

    void set(double position) {
        this.position = position;
    }

    void setVelocity(double velocity) {
        this.velocity = velocity;
    }
}
