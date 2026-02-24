package ca.frc6390.athena.mechanisms;

import java.util.function.Function;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.context.RobotScopedContext;
import ca.frc6390.athena.core.input.TypedInputContext;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.networktables.NtScope;

/**
 * Read-only view of the composite that constraints can use to inspect child mechanisms.
 *
 * @param <SP> setpoint tuple type produced by the superstate enum
 */
public interface SuperstructureContext<SP> extends TypedInputContext, RobotScopedContext {

    SP setpoint();

    /**
     * Returns the robot core associated with the superstructure's child mechanisms.
     */
    RobotCore<?> robotCore();

    /**
     * Name used for default NetworkTables scoping.
     */
    default String superstructureName() {
        return "Superstructure";
    }

    @Override
    default NtScope nt() {
        return AthenaNT.scope("Superstructures")
                .scope(superstructureName())
                .scope("NetworkTables");
    }

    <E extends Enum<E> & SetpointProvider<Double>> StatefulLike<E> mechanism(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(Function<SP, CS> mapper);

    /**
     * Returns a typed accessor for child mechanisms and nested superstructures.
     */
    SuperstructureMechanismsView<SP> mechanisms();

}
