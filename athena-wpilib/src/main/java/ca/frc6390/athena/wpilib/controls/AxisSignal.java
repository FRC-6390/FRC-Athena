package ca.frc6390.athena.wpilib.controls;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Immutable processing pipeline for one controller axis.
 */
public final class AxisSignal {
    private final DoubleSupplier source;
    private final double deadband;
    private final boolean squared;
    private final boolean inverted;

    AxisSignal(DoubleSupplier source) {
        this(source, 0.0, false, false);
    }

    private AxisSignal(DoubleSupplier source, double deadband, boolean squared, boolean inverted) {
        this.source = Objects.requireNonNull(source, "source");
        this.deadband = sanitizeDeadband(deadband);
        this.squared = squared;
        this.inverted = inverted;
    }

    public AxisSignal deadband(double deadband) {
        return new AxisSignal(source, deadband, squared, inverted);
    }

    public AxisSignal squared() {
        return new AxisSignal(source, deadband, true, inverted);
    }

    public AxisSignal inverted() {
        return new AxisSignal(source, deadband, squared, true);
    }

    public DoubleSupplier toSupplier() {
        return this::value;
    }

    private double value() {
        double value = source.getAsDouble();
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        value = Math.max(-1.0, Math.min(1.0, value));
        if (Math.abs(value) <= deadband) {
            value = 0.0;
        } else {
            value = value > 0.0
                    ? (value - deadband) / (1.0 - deadband)
                    : (value + deadband) / (1.0 - deadband);
        }
        if (squared) {
            value = Math.copySign(value * value, value);
        }
        return inverted ? -value : value;
    }

    private static double sanitizeDeadband(double requested) {
        if (!Double.isFinite(requested)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(0.99, Math.abs(requested)));
    }
}
