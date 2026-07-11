package ca.frc6390.athena.wpilib.controls;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Immutable processing pipeline for one controller axis.
 */
public final class AxisSignal {
    private final Gamepad owner;
    private final String name;
    private final DoubleSupplier source;
    private final double deadband;
    private final boolean squared;
    private final boolean inverted;

    AxisSignal(DoubleSupplier source) {
        this(null, "axis", source, 0.0, false, false);
    }

    AxisSignal(Gamepad owner, String name, DoubleSupplier source) {
        this(owner, name, source, 0.0, false, false);
    }

    private AxisSignal(
            Gamepad owner,
            String name,
            DoubleSupplier source,
            double deadband,
            boolean squared,
            boolean inverted) {
        this.owner = owner;
        this.name = name == null || name.isBlank() ? "axis" : name;
        this.source = Objects.requireNonNull(source, "source");
        this.deadband = sanitizeDeadband(deadband);
        this.squared = squared;
        this.inverted = inverted;
    }

    public AxisSignal deadband(double deadband) {
        return new AxisSignal(owner, name, source, deadband, squared, inverted);
    }

    public AxisSignal squared() {
        return new AxisSignal(owner, name, source, deadband, true, inverted);
    }

    public AxisSignal inverted() {
        return new AxisSignal(owner, name, source, deadband, squared, true);
    }

    public DoubleSupplier toSupplier() {
        return this::value;
    }

    /** Returns a control signal active above the threshold. */
    public ControlSignal above(double threshold) {
        return above(threshold, threshold);
    }

    /** Returns a control signal with separate engage and release thresholds. */
    public ControlSignal above(double engage, double release) {
        requireFinite(engage, "engage");
        requireFinite(release, "release");
        if (release > engage) {
            throw new IllegalArgumentException("release must be less than or equal to engage");
        }
        return threshold("above", current -> current ? value() > release : value() > engage);
    }

    /** Returns a control signal active below the threshold. */
    public ControlSignal below(double threshold) {
        return below(threshold, threshold);
    }

    /** Returns a control signal with separate engage and release thresholds. */
    public ControlSignal below(double engage, double release) {
        requireFinite(engage, "engage");
        requireFinite(release, "release");
        if (release < engage) {
            throw new IllegalArgumentException("release must be greater than or equal to engage");
        }
        return threshold("below", current -> current ? value() < release : value() < engage);
    }

    /** Returns a control signal active outside the supplied absolute magnitude. */
    public ControlSignal outside(double magnitude) {
        requireMagnitude(magnitude);
        return new ControlSignal(owner, name + ".outside[" + magnitude + "]", context -> Math.abs(value()) > magnitude);
    }

    /** Returns a control signal active inside the supplied absolute magnitude. */
    public ControlSignal inside(double magnitude) {
        requireMagnitude(magnitude);
        return new ControlSignal(owner, name + ".inside[" + magnitude + "]", context -> Math.abs(value()) < magnitude);
    }

    private ControlSignal threshold(String operation, java.util.function.Predicate<Boolean> evaluator) {
        final class State {
            boolean active;

            boolean evaluate() {
                active = evaluator.test(active);
                return active;
            }
        }
        State state = new State();
        return new ControlSignal(owner, name + "." + operation, context -> state.evaluate());
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

    private static void requireFinite(double value, String parameter) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(parameter + " must be finite");
        }
    }

    private static void requireMagnitude(double magnitude) {
        requireFinite(magnitude, "magnitude");
        if (magnitude < 0.0 || magnitude > 1.0) {
            throw new IllegalArgumentException("magnitude must be between 0 and 1");
        }
    }
}
