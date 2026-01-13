package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;

/**
 * Shared view used to access child mechanisms from a superstructure context.
 */
public interface SuperstructureMechanismsView<SP> {

    <E extends Enum<E> & SetpointProvider<Double>> StatefulArmMechanism<E> arm(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulElevatorMechanism<E> elevator(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulTurretMechanism<E> turret(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> generic(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(
            Function<SP, CS> mapper);

    boolean input(String key);

    java.util.function.BooleanSupplier inputSupplier(String key);

    List<Mechanism> all();
}
