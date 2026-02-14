package ca.frc6390.athena.mechanisms;

import java.util.function.Function;
import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;
import ca.frc6390.athena.core.input.TypedInputContext;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Read-only view of the composite that constraints can use to inspect child mechanisms.
 *
 * @param <SP> setpoint tuple type produced by the superstate enum
 */
public interface SuperstructureContext<SP> extends TypedInputContext {

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
     * Returns the base setpoint mapped from the current superstate setpoint.
     */
    <E extends Enum<E> & StateMachine.SetpointProvider<Double>> double mappedSetpoint(Function<SP, E> mapper);
}
