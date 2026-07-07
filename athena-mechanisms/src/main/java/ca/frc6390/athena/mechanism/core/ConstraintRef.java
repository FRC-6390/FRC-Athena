package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RangeRef;
import java.util.Objects;

/**
 * Control-local limits that shape how a controller may generate motion.
 */
public record ConstraintRef(
        RangeRef position,
        RangeRef velocity,
        RangeRef acceleration) {
    public static ConstraintRef none() {
        return new ConstraintRef(null, null, null);
    }

    public ConstraintRef position(double minimum, double maximum) {
        return position(RangeRef.of(minimum, maximum));
    }

    public ConstraintRef position(RangeRef position) {
        return new ConstraintRef(Objects.requireNonNull(position, "position"), velocity, acceleration);
    }

    public ConstraintRef velocity(double maximum) {
        return velocity(-Math.abs(maximum), Math.abs(maximum));
    }

    public ConstraintRef velocity(double minimum, double maximum) {
        return velocity(RangeRef.of(minimum, maximum));
    }

    public ConstraintRef velocity(RangeRef velocity) {
        return new ConstraintRef(position, Objects.requireNonNull(velocity, "velocity"), acceleration);
    }

    public ConstraintRef acceleration(double maximum) {
        return acceleration(-Math.abs(maximum), Math.abs(maximum));
    }

    public ConstraintRef acceleration(double minimum, double maximum) {
        return acceleration(RangeRef.of(minimum, maximum));
    }

    public ConstraintRef acceleration(RangeRef acceleration) {
        return new ConstraintRef(position, velocity, Objects.requireNonNull(acceleration, "acceleration"));
    }
}
