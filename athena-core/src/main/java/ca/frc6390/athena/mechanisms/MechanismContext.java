package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.context.RobotScopedContext;
import ca.frc6390.athena.core.input.TypedInputContext;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view used by mechanism state hooks.
 *
 * @param <T> mechanism type
 * @param <E> state enum type
 */
public interface MechanismContext<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>>
        extends TypedInputContext, RobotScopedContext {

    T mechanism();

    E state();

    double setpoint();

    default RobotCore<?> robotCore() {
        RobotCore<?> core = mechanism().getRobotCore();
        return core != null ? core : RobotCore.activeInstance();
    }

    /**
     * Disables a named control loop on the owning mechanism.
     */
    default void disableControlLoop(String name) {
        mechanism().loops().disable(name);
    }

    /**
     * Enables a named control loop on the owning mechanism.
     */
    default void enableControlLoop(String name) {
        mechanism().loops().enable(name);
    }

    /**
     * Returns whether a named control loop is enabled on the owning mechanism.
     */
    default boolean isControlLoopEnabled(String name) {
        return mechanism().loops().enabled(name);
    }
}
