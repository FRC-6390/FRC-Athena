package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot intent for a mechanism.
 */
public interface Action extends DeviceAction {
    @Override
    default void apply(ActionContext context) {
        request();
    }

    default void request() {
        ActionRequests.request(this);
    }

    default Action until(ActionCondition condition) {
        return new Conditional(this, condition, null);
    }

    default Action until(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean());
    }

    /**
     * Completes this position or velocity control action when its configured
     * feedback is within the supplied tolerance of the requested target.
     *
     * @param tolerance maximum absolute target error in the control's configured units
     * @return feedback-completing action
     */
    default Action untilWithin(double tolerance) {
        return Actions.untilWithin(this, tolerance);
    }

    default Action then(Action next) {
        return new Then(this, next);
    }

    record Conditional(Action action, ActionCondition condition, Action next) implements Action {
        public Conditional {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(condition, "condition");
        }

        @Override
        public Action then(Action next) {
            return new Conditional(action, condition, Objects.requireNonNull(next, "next"));
        }
    }

    record Then(Action action, Action next) implements Action {
        public Then {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(next, "next");
        }
    }

}
