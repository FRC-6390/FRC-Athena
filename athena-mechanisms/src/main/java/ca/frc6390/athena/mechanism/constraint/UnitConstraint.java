package ca.frc6390.athena.mechanism.constraint;

import edu.wpi.first.units.Units;

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

    /**
     * Returns the constraint for a binding of the given mode. A position binding uses {@link #expressedIn(Unit)}.
     * A velocity binding's targets are rates, so the default expresses the limits in {@code unit} per second
     * (a range bounds the velocity target). Motion limits override this: in velocity mode they are an
     * acceleration and its rate (jerk), not a velocity and an acceleration.
     *
     * @param unit unit of the control binding (the position unit for a velocity binding)
     * @param velocityControl true when the binding is a velocity control
     * @return converted constraint
     */
    default Constraint<Double> expressedIn(Unit unit, boolean velocityControl) {
        return velocityControl
                ? expressedIn(ca.frc6390.athena.hardware.units.UnitConversions.perSecond(unit))
                : expressedIn(unit);
    }

    @Override
    default ConstraintResult<Double> evaluate(ConstraintContext<Double> context) {
        throw new IllegalStateException("Typed constraints must be attached to a unit-typed control binding "
                + "(Controls.position(motor, Units.Rotations).followers(unit)) before evaluation.");
    }
}
