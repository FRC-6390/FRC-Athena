package ca.frc6390.athena.core;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.mechanisms.OutputType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;

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

    /**
     * Sectioned interaction helper for already-built mechanisms/superstructures.
     */
    default void robotMechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        robotMechanisms().use(section);
    }

    /**
     * Returns the elapsed time (seconds) since the currently running core control loop last ran.
     * If unavailable, returns {@link Double#NaN}.
     */
    default double controlLoopDtSeconds() {
        return Double.NaN;
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

    /**
     * Returns a named PID controller registered in {@link RobotCoreHooks.HooksSection#pidProfile}.
     */
    PIDController pid(String name);

    /**
     * Returns a named feedforward model registered in {@link RobotCoreHooks.HooksSection#feedforwardProfile}.
     */
    SimpleMotorFeedforward feedforward(String name);

    /**
     * Returns the declared output space for a PID profile.
     */
    OutputType pidOutputType(String name);

    /**
     * Returns the declared output space for a feedforward profile.
     */
    OutputType feedforwardOutputType(String name);

    default double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    default double percentToVolts(double percent) {
        return percent * batteryVoltage();
    }

    default double voltsToPercent(double volts) {
        double battery = batteryVoltage();
        if (!Double.isFinite(battery) || Math.abs(battery) < 1e-9) {
            return 0.0;
        }
        return volts / battery;
    }

    default double convertOutput(OutputType from, OutputType to, double value) {
        OutputType resolvedFrom = from != null ? from : OutputType.VOLTAGE;
        OutputType resolvedTo = to != null ? to : resolvedFrom;
        if (resolvedFrom == resolvedTo) {
            return value;
        }
        if (resolvedFrom == OutputType.PERCENT && resolvedTo == OutputType.VOLTAGE) {
            return percentToVolts(value);
        }
        if (resolvedFrom == OutputType.VOLTAGE && resolvedTo == OutputType.PERCENT) {
            return voltsToPercent(value);
        }
        throw new IllegalArgumentException(
                "Unsupported output conversion from " + resolvedFrom + " to " + resolvedTo + " for RobotCore context");
    }

    default double pidOut(String name, double measurement, double setpoint) {
        PIDController controller = pid(name);
        if (controller == null) {
            return 0.0;
        }
        return controller.calculate(measurement, setpoint);
    }

    default double pidOut(String name, double measurement, double setpoint, OutputType outputType) {
        double raw = pidOut(name, measurement, setpoint);
        return convertOutput(pidOutputType(name), outputType, raw);
    }

    default double feedforwardOut(String name, double velocity) {
        SimpleMotorFeedforward ff = feedforward(name);
        if (ff == null) {
            return 0.0;
        }
        return ff.calculate(velocity);
    }

    default double feedforwardOut(String name, double velocity, OutputType outputType) {
        double raw = feedforwardOut(name, velocity);
        return convertOutput(feedforwardOutputType(name), outputType, raw);
    }

    default double feedforwardOut(String name, double currentVelocity, double nextVelocity) {
        SimpleMotorFeedforward ff = feedforward(name);
        if (ff == null) {
            return 0.0;
        }
        return ff.calculateWithVelocities(currentVelocity, nextVelocity);
    }

    default double feedforwardOut(String name, double currentVelocity, double nextVelocity, OutputType outputType) {
        double raw = feedforwardOut(name, currentVelocity, nextVelocity);
        return convertOutput(feedforwardOutputType(name), outputType, raw);
    }
}
