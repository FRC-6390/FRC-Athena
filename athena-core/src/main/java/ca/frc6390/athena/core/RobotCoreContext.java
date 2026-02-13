package ca.frc6390.athena.core;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.logging.TelemetryRegistry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Read-only lifecycle context passed to {@link RobotCoreHooks} bindings.
 *
 * @param <T> drivetrain type owned by the robot core
 */
public interface RobotCoreContext<T extends RobotDrivetrain<T>> {

    RobotCore<T> robotCore();

    RobotCoreHooks.Phase phase();

    default T drivetrain() {
        return robotCore().getDrivetrain();
    }

    default RobotLocalization<?> localization() {
        return robotCore().getLocalization();
    }

    default RobotVision vision() {
        return robotCore().getVision();
    }

    default RobotAuto autos() {
        return robotCore().getAutos();
    }

    default RobotCopilot copilot() {
        return robotCore().getCopilot();
    }

    default RobotNetworkTables networkTables() {
        return robotCore().networkTables();
    }

    default TelemetryRegistry telemetry() {
        return robotCore().telemetry();
    }

    default RobotMechanisms robotMechanisms() {
        return robotCore().getMechanisms();
    }

    boolean input(String key);

    BooleanSupplier inputSupplier(String key);

    default boolean boolVal(String key) {
        return input(key);
    }

    default BooleanSupplier boolValSupplier(String key) {
        return inputSupplier(key);
    }

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    default double doubleVal(String key) {
        return doubleInput(key);
    }

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

    <V> V objectInput(String key, Class<V> type);

    <V> Supplier<V> objectInputSupplier(String key, Class<V> type);
}
