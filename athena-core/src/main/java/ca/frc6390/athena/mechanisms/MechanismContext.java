package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view used by mechanism state hooks.
 *
 * @param <T> mechanism type
 * @param <E> state enum type
 */
public interface MechanismContext<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {

    T mechanism();

    E state();

    double baseSetpoint();

    default RobotCore<?> robotCore() {
        return mechanism().getRobotCore();
    }

    boolean input(String key);

    BooleanSupplier inputSupplier(String key);

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    <V> V objectInput(String key, Class<V> type);

    <V> Supplier<V> objectInputSupplier(String key, Class<V> type);
}
