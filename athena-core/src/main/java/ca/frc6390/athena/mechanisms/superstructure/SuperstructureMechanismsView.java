package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism.StatefulFlywheelMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Shared view used to access child mechanisms from a superstructure context.
 */
public interface SuperstructureMechanismsView<SP> {

    <E extends Enum<E> & SetpointProvider<Double>> StatefulArmMechanism<E> arm(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulElevatorMechanism<E> elevator(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulTurretMechanism<E> turret(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulFlywheelMechanism<E> flywheel(Function<SP, E> mapper);

    <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> generic(Function<SP, E> mapper);

    <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(
            Function<SP, CS> mapper);

    boolean input(String key);

    java.util.function.BooleanSupplier inputSupplier(String key);

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    int intVal(String key);

    IntSupplier intValSupplier(String key);

    String stringVal(String key);

    Supplier<String> stringValSupplier(String key);

    Pose2d pose2dVal(String key);

    Supplier<Pose2d> pose2dValSupplier(String key);

    Pose3d pose3dVal(String key);

    Supplier<Pose3d> pose3dValSupplier(String key);

    <T> T objectInput(String key, Class<T> type);

    <T> Supplier<T> objectInputSupplier(String key, Class<T> type);

    List<Mechanism> all();
}
