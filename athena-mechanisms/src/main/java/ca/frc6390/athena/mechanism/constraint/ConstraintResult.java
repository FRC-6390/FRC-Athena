package ca.frc6390.athena.mechanism.constraint;

import java.util.Objects;

/**
 * Result of evaluating a constraint.
 *
 * @param <T> constrained value type
 */
public sealed interface ConstraintResult<T>
        permits ConstraintResult.Allowed, ConstraintResult.Corrected, ConstraintResult.Rejected {
    T requested();

    default boolean accepted() {
        return !(this instanceof Rejected<?>);
    }

    default T value() {
        if (this instanceof Allowed<T> allowed) {
            return allowed.value();
        }
        if (this instanceof Corrected<T> corrected) {
            return corrected.value();
        }
        return requested();
    }

    record Allowed<T>(T value) implements ConstraintResult<T> {
        public Allowed {
            Objects.requireNonNull(value, "value");
        }

        @Override
        public T requested() {
            return value;
        }
    }

    record Corrected<T>(T requested, T value) implements ConstraintResult<T> {
        public Corrected {
            Objects.requireNonNull(requested, "requested");
            Objects.requireNonNull(value, "value");
        }
    }

    record Rejected<T>(T requested) implements ConstraintResult<T> {
        public Rejected {
            Objects.requireNonNull(requested, "requested");
        }
    }
}
