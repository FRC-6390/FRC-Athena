package ca.frc6390.athena.mechanism.constraint;

import ca.frc6390.athena.mechanism.core.MechanismContext;
import java.util.Objects;

/**
 * Current and requested values supplied to a constraint.
 *
 * @param current current mechanism value
 * @param requested requested mechanism value
 * @param runtime current mechanism runtime context
 * @param <T> constrained value type
 */
public record ConstraintContext<T>(
        T current,
        T requested,
        MechanismContext runtime) {

    public ConstraintContext {
        Objects.requireNonNull(current, "current");
        Objects.requireNonNull(requested, "requested");
        runtime = runtime == null ? MechanismContext.empty() : runtime;
    }

    public ConstraintContext<T> requested(T value) {
        return new ConstraintContext<>(current, Objects.requireNonNull(value, "value"), runtime);
    }
}
