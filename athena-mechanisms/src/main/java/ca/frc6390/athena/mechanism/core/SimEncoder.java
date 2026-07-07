package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RuntimeEncoder;
import java.util.Objects;

final class SimEncoder implements RuntimeEncoder {
    private final RuntimeEncoder delegate;
    private double position;
    private double velocity;

    SimEncoder(RuntimeEncoder delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.position = delegate.position();
        this.velocity = delegate.velocity();
    }

    @Override
    public double position() {
        return position;
    }

    @Override
    public double absolutePosition() {
        return position;
    }

    @Override
    public double velocity() {
        return velocity;
    }

    @Override
    public void set(double position) {
        this.position = position;
        delegate.set(position);
    }

    @Override
    public void setVelocity(double velocity) {
        this.velocity = velocity;
        delegate.setVelocity(velocity);
    }

    @Override
    public void syncTo(RuntimeEncoder source) {
        RuntimeEncoder.super.syncTo(source);
    }
}
