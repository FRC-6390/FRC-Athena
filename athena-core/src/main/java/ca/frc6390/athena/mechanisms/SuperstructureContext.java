package ca.frc6390.athena.mechanisms;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Read-only view of the composite that constraints can use to inspect child mechanisms.
 *
 * @param <SP> setpoint tuple type produced by the superstate enum
 */
public interface SuperstructureContext<SP> {

    SP setpoint();

    /**
     * Returns the robot core associated with the superstructure's child mechanisms.
     */
    RobotCore<?> robotCore();

    /**
     * Returns the global robot-wide mechanisms view (lookup by name/config/type).
     *
     * <p>This is the recommended way to shorten long nested access chains inside hooks/constraints:
     * {@code ctx.robotMechanisms().turret(Turret.TURRET_CONFIG).atSetpoint()}.</p>
     */
    default RobotMechanisms robotMechanisms() {
        RobotCore<?> core = robotCore();
        if (core == null) {
            throw new IllegalStateException("No RobotCore available in superstructure context");
        }
        return core.getMechanisms();
    }

    /**
     * Sectioned interaction helper for other already-built mechanisms/superstructures.
     */
    default void robotMechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        robotMechanisms().use(section);
    }

    <E extends Enum<E> & SetpointProvider<Double>> StatefulLike<E> mechanism(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(Function<SP, CS> mapper);

    /**
     * Returns a typed accessor for child mechanisms and nested superstructures.
     */
    SuperstructureMechanismsView<SP> getMechanisms();

    /**
     * Returns the value of a named external input (added via the superstructure config).
     */
    boolean input(String key);

    default boolean boolVal(String key) {
        return input(key);
    }

    /**
     * Returns the value of a named double input (added via the superstructure config).
     */
    double doubleInput(String key);

    default double doubleVal(String key) {
        return doubleInput(key);
    }

    /**
     * Returns the supplier for a named double input.
     */
    DoubleSupplier doubleInputSupplier(String key);

    default DoubleSupplier doubleValSupplier(String key) {
        return doubleInputSupplier(key);
    }

    int intVal(String key);

    IntSupplier intValSupplier(String key);

    String stringVal(String key);

    Supplier<String> stringValSupplier(String key);

    Pose2d pose2dVal(String key);

    Supplier<Pose2d> pose2dValSupplier(String key);

    Pose3d pose3dVal(String key);

    Supplier<Pose3d> pose3dValSupplier(String key);

    /**
     * Returns a named object input using the requested type.
     */
    <T> T objectInput(String key, Class<T> type);

    /**
     * Returns the supplier for a named object input.
     */
    <T> Supplier<T> objectInputSupplier(String key, Class<T> type);

    /**
     * Returns the base setpoint mapped from the current superstate setpoint.
     */
    <E extends Enum<E> & StateMachine.SetpointProvider<Double>> double mappedSetpoint(Function<SP, E> mapper);
}
