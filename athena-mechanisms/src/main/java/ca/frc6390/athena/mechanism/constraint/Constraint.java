package ca.frc6390.athena.mechanism.constraint;

/**
 * Evaluates and optionally corrects one requested mechanism value.
 *
 * @param <T> constrained value type
 */
@FunctionalInterface
public interface Constraint<T> {
    ConstraintResult<T> evaluate(ConstraintContext<T> context);

    /** Returns the control stage governed by this constraint. */
    default ConstraintStage stage() {
        return ConstraintStage.TARGET;
    }
}
