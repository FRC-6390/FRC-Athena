package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.Range;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot intent for a mechanism.
 */
public interface State {
    default State until(StateCondition condition) {
        return new Conditional(this, condition, null);
    }

    default State until(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean());
    }

    default State then(State next) {
        return new Then(this, next);
    }

    default State clamp(Range range) {
        return new Clamped(this, range);
    }

    record Conditional(State state, StateCondition condition, State next) implements State {
        public Conditional {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(condition, "condition");
        }

        @Override
        public State then(State next) {
            return new Conditional(state, condition, Objects.requireNonNull(next, "next"));
        }
    }

    record Then(State state, State next) implements State {
        public Then {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(next, "next");
        }
    }

    record Clamped(State state, Range range) implements State {
        public Clamped {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(range, "range");
        }
    }
}
