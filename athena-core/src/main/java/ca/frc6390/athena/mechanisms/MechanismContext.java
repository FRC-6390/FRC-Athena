package ca.frc6390.athena.mechanisms;

import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;
import ca.frc6390.athena.core.input.TypedInputContext;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view used by mechanism state hooks.
 *
 * @param <T> mechanism type
 * @param <E> state enum type
 */
public interface MechanismContext<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>>
        extends TypedInputContext {

    T mechanism();

    E state();

    double baseSetpoint();

    default RobotCore<?> robotCore() {
        RobotCore<?> core = mechanism().getRobotCore();
        return core != null ? core : RobotCore.getActiveInstance();
    }

    /**
     * Returns the global robot-wide mechanisms view (lookup by name/config/type).
     */
    default RobotMechanisms robotMechanisms() {
        RobotCore<?> core = robotCore();
        if (core == null) {
            throw new IllegalStateException("No RobotCore available in mechanism context");
        }
        return core.getMechanisms();
    }

    /**
     * Sectioned interaction helper for other already-built mechanisms/superstructures.
     */
    default void robotMechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        robotMechanisms().use(section);
    }

    /**
     * Disables a named control loop on the owning mechanism.
     */
    default void disableControlLoop(String name) {
        mechanism().disableControlLoop(name);
    }

    /**
     * Enables a named control loop on the owning mechanism.
     */
    default void enableControlLoop(String name) {
        mechanism().enableControlLoop(name);
    }

    /**
     * Returns whether a named control loop is enabled on the owning mechanism.
     */
    default boolean isControlLoopEnabled(String name) {
        return mechanism().isControlLoopEnabled(name);
    }
}
