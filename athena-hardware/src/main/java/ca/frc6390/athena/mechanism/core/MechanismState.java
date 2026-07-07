package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RangeRef;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot intent for a mechanism.
 */
public interface MechanismState {
    /**
     * Runs this state until a condition is true.
     *
     * @param condition condition
     * @return conditional state
     */
    default MechanismState until(StateCondition condition) {
        return new Conditional(this, condition, null);
    }

    /**
     * Runs this state until a no-argument condition is true.
     *
     * @param condition condition
     * @return conditional state
     */
    default MechanismState until(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean());
    }

    /**
     * Requests a transition after this state completes.
     *
     * @param next next state
     * @return state with transition target
     */
    default MechanismState then(MechanismState next) {
        return new Then(this, next);
    }

    /**
     * Applies a mechanism range constraint to this state.
     *
     * @param range range
     * @return constrained state
     */
    default MechanismState clamp(RangeRef range) {
        return new Clamped(this, range);
    }

    /**
     * Conditional wrapper created by direct state chaining.
     */
    record Conditional(
            MechanismState state,
            StateCondition condition,
            MechanismState next) implements MechanismState {
        public Conditional {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(condition, "condition");
        }

        @Override
        public MechanismState then(MechanismState next) {
            return new Conditional(state, condition, Objects.requireNonNull(next, "next"));
        }
    }

    /**
     * Transition wrapper created by direct state chaining.
     */
    record Then(MechanismState state, MechanismState next) implements MechanismState {
        public Then {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(next, "next");
        }
    }

    /**
     * Range-constrained wrapper created by direct state chaining.
     */
    record Clamped(MechanismState state, RangeRef range) implements MechanismState {
        public Clamped {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(range, "range");
        }
    }
}
