package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCore;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.RobotController;

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
        RobotCore<?> core = mechanism().getRobotCore();
        return core != null ? core : RobotCore.getActiveInstance();
    }

    boolean input(String key);

    BooleanSupplier inputSupplier(String key);

    double doubleInput(String key);

    DoubleSupplier doubleInputSupplier(String key);

    <V> V objectInput(String key, Class<V> type);

    <V> Supplier<V> objectInputSupplier(String key, Class<V> type);

    default boolean usesVoltage() {
        return mechanism().isUseVoltage();
    }

    default OutputType outputType() {
        return mechanism().getOutputType();
    }

    default double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    /**
     * Converts a percent output (-1..1) into the mechanism's output space.
     * If the mechanism uses voltage, this returns volts. Otherwise it returns percent.
     */
    default double percentToOutput(double percent) {
        if (usesVoltage()) {
            return percent * batteryVoltage();
        }
        return percent;
    }

    /**
     * Converts volts into the mechanism's output space.
     * If the mechanism uses voltage, this returns volts. Otherwise it returns percent.
     */
    default double voltsToOutput(double volts) {
        if (usesVoltage()) {
            return volts;
        }
        double vbat = batteryVoltage();
        if (!Double.isFinite(vbat) || vbat == 0.0) {
            return 0.0;
        }
        return volts / vbat;
    }

    /**
     * Converts a value from the given output type into the mechanism's output space.
     */
    default double toOutput(OutputType type, double value) {
        if (type == null) {
            return value;
        }
        if (type == outputType()) {
            return value;
        }
        return switch (type) {
            case VOLTAGE -> voltsToOutput(value);
            case PERCENT -> percentToOutput(value);
            case POSITION, VELOCITY -> value;
        };
    }

    /**
     * Converts a mechanism output value into percent (-1..1).
     */
    default double outputToPercent(double output) {
        if (!usesVoltage()) {
            return output;
        }
        double vbat = batteryVoltage();
        if (!Double.isFinite(vbat) || vbat == 0.0) {
            return 0.0;
        }
        return output / vbat;
    }

    /**
     * Converts a mechanism output value into volts.
     */
    default double outputToVolts(double output) {
        if (usesVoltage()) {
            return output;
        }
        return output * batteryVoltage();
    }

    /**
     * Clamps an output value to the valid range for the mechanism.
     * For percent output: [-1, 1]. For voltage output: [-batteryVoltage, batteryVoltage].
     */
    default double clampOutput(double output) {
        if (usesVoltage()) {
            double vbat = batteryVoltage();
            if (!Double.isFinite(vbat) || vbat <= 0.0) {
                return 0.0;
            }
            return Math.max(-vbat, Math.min(vbat, output));
        }
        return Math.max(-1.0, Math.min(1.0, output));
    }

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
