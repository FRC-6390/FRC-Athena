package ca.frc6390.athena.mechanism.constraint;

/**
 * Evaluates and optionally corrects one requested mechanism value.
 *
 * @param <T> constrained value type
 */
@FunctionalInterface
public interface Constraint<T> {
    /** Kind used when a constraint does not name itself. */
    String DEFAULT_KIND = "constraint";

    ConstraintResult<T> evaluate(ConstraintContext<T> context);

    /** Returns the kind of this constraint, named in telemetry when it rejects a request. */
    default String kind() {
        return DEFAULT_KIND;
    }

    /** Returns the control stage governed by this constraint. */
    default ConstraintStage stage() {
        return ConstraintStage.TARGET;
    }
}
