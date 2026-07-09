package ca.frc6390.athena.hardware.runtime;

import java.util.Objects;

/**
 * Runtime action declared against device dependencies and executed with an action context.
 */
@FunctionalInterface
public interface ActionBinding {
    /**
     * Applies this action using runtime access.
     *
     * @param context runtime action context
     */
    void apply(ActionContext context);

    /**
     * Creates an action from ordinary code that does not need runtime ref access.
     *
     * @param action action to run
     * @return action ref
     */
    static ActionBinding run(Runnable action) {
        Objects.requireNonNull(action, "action");
        return context -> action.run();
    }
}
