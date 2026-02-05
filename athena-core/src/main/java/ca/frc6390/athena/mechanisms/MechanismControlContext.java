package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCore;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Read-only view used by custom control loops.
 *
 * @param <T> mechanism type
 */
public interface MechanismControlContext<T extends Mechanism> {

    T mechanism();

    /**
     * Base setpoint for the mechanism, excluding any runtime overrides.
     */
    double baseSetpoint();

    /**
     * Active state for stateful mechanisms, or {@code null} if none exists.
     */
    Enum<?> state();

    default RobotCore<?> robotCore() {
        return mechanism().getRobotCore();
    }

    boolean input(String key);

    BooleanSupplier inputSupplier(String key);

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    <V> V objectInput(String key, Class<V> type);

    <V> Supplier<V> objectInputSupplier(String key, Class<V> type);

    default void disableControlLoop(String name) {
        mechanism().disableControlLoop(name);
    }

    default void enableControlLoop(String name) {
        mechanism().enableControlLoop(name);
    }

    default boolean isControlLoopEnabled(String name) {
        return mechanism().isControlLoopEnabled(name);
    }
}
