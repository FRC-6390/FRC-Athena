package ca.frc6390.athena.core;

import ca.frc6390.athena.core.context.RobotScopedContext;
import ca.frc6390.athena.core.input.TypedInputContext;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.OutputConversions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Read-only lifecycle context passed to {@link RobotCoreHooks} bindings.
 *
 * @param <T> drivetrain type owned by the robot core
 */
public interface RobotCoreContext<T extends RobotDrivetrain<T>>
        extends TypedInputContext, RobotScopedContext {

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

    /**
     * Returns the elapsed time (seconds) since the currently running core control loop last ran.
     * If unavailable, returns {@link Double#NaN}.
     */
    default double controlLoopDtSeconds() {
        return Double.NaN;
    }

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
        return OutputConversions.percentToVolts(percent, batteryVoltage());
    }

    default double voltsToPercent(double volts) {
        return OutputConversions.voltsToPercent(volts, batteryVoltage());
    }

    default double convertOutput(OutputType from, OutputType to, double value) {
        return OutputConversions.convert(
                from != null ? from : OutputType.VOLTAGE,
                to,
                value,
                batteryVoltage());
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
