package ca.frc6390.athena.mechanism.model;

import java.util.function.DoubleSupplier;

/**
 * Compares the binding's primary feedback with a second encoder on the same mechanism, scaled by the
 * second encoder's gear ratio. Created by {@code ControlBinding.slipCheck(...)}.
 */
public final class SlipCheck {
    private final DoubleSupplier primary;
    private final DoubleSupplier secondaryMechanism;
    private final double tolerance;

    public SlipCheck(DoubleSupplier primary, DoubleSupplier secondaryMechanism, double tolerance) {
        if (!(tolerance >= 0.0) || !Double.isFinite(tolerance)) {
            throw new IllegalArgumentException("Slip tolerance must be finite and non-negative.");
        }
        this.primary = java.util.Objects.requireNonNull(primary, "primary");
        this.secondaryMechanism = java.util.Objects.requireNonNull(secondaryMechanism, "secondaryMechanism");
        this.tolerance = tolerance;
    }

    /** Returns the disagreement between the two feedbacks, in binding units. */
    public double slip() {
        return primary.getAsDouble() - secondaryMechanism.getAsDouble();
    }

    /** Returns true when the feedbacks disagree by more than the tolerance. */
    public boolean slipping() {
        double slip = slip();
        return Double.isFinite(slip) && Math.abs(slip) > tolerance;
    }

    public double tolerance() {
        return tolerance;
    }
}
