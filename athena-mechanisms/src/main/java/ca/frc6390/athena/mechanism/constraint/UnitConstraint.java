package ca.frc6390.athena.mechanism.constraint;

import edu.wpi.first.units.Unit;

/**
 * A {@code Double} constraint whose limits were declared with WPILib measures and are not yet tied to a
 * control binding's unit. {@code ControlBinding} converts it with {@link #expressedIn(Unit)} when the
 * binding is declared, so one typed constraint works on any binding of the same dimension. Evaluating
 * an unconverted typed constraint is a programming error and throws.
 */
public interface UnitConstraint extends Constraint<Double> {
    /**
     * Returns an ordinary constraint whose limits are plain values in {@code unit}.
     *
     * @param unit unit of the control binding the constraint is attached to
     * @return converted constraint
     * @throws IllegalArgumentException when the constraint's dimension differs from {@code unit}
     */
    Constraint<Double> expressedIn(Unit unit);

    @Override
    default ConstraintResult<Double> evaluate(ConstraintContext<Double> context) {
        throw new IllegalStateException("Typed constraints must be attached to a unit-typed control binding "
                + "(Controls.position(motor, unit)) before evaluation.");
    }
}
