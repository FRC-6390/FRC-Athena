package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RangeRef;
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
        return States.until(this, condition);
    }

    /**
     * Runs this state until a no-argument condition is true.
     *
     * @param condition condition
     * @return conditional state
     */
    default MechanismState until(BooleanSupplier condition) {
        return States.until(this, condition);
    }

    /**
     * Requests a transition after this state completes.
     *
     * @param next next state
     * @return state with transition target
     */
    default MechanismState then(MechanismState next) {
        return States.then(this, next);
    }

    /**
     * Applies a mechanism range constraint to this state.
     *
     * @param range range
     * @return constrained state
     */
    default MechanismState clamp(RangeRef range) {
        return States.clamp(this, range);
    }
}
