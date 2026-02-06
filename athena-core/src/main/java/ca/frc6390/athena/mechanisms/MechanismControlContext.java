package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCore;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;

/**
 * Read-only view used by custom control loops.
 *
 * @param <T> mechanism type
 */
public interface MechanismControlContext<T extends Mechanism> {

    enum PositionUnit {
        ROTATIONS,
        RADIANS,
        DEGREES,
        ENCODER_UNITS
    }

    enum VelocityUnit {
        ROTATIONS_PER_SEC,
        RAD_PER_SEC,
        DEG_PER_SEC,
        ENCODER_UNITS_PER_SEC
    }

    T mechanism();

    /**
     * Returns the elapsed time (seconds) since this control loop last ran.
     * If unavailable, returns {@link Double#NaN}.
     */
    default double controlLoopDtSeconds() {
        return Double.NaN;
    }

    /**
     * Returns the motor group for the mechanism, if any.
     */
    default MotorControllerGroup motorGroup() {
        return mechanism().getMotorGroup();
    }

    /**
     * Returns the individual motors for the mechanism, if any.
     */
    default MotorController[] motors() {
        MotorControllerGroup group = motorGroup();
        return group != null ? group.getControllers() : new MotorController[0];
    }

    /**
     * Returns the primary encoder for the mechanism, if any.
     */
    default Encoder encoder() {
        return mechanism().getEncoder();
    }

    /**
     * Returns the encoder group for the mechanism's motor group, if any.
     */
    default EncoderGroup encoderGroup() {
        MotorControllerGroup group = motorGroup();
        return group != null ? group.getEncoderGroup() : null;
    }

    /**
     * Returns the encoders for the mechanism's motor group, if any.
     */
    default Encoder[] encoders() {
        EncoderGroup group = encoderGroup();
        return group != null ? group.getEncoders() : new Encoder[0];
    }

    /**
     * Converts a raw encoder position (rotations) into mechanism-space units
     * using the encoder's gear ratio, conversion, and offsets.
     */
    default double encoderRawToMechanismPosition(double rawRotations) {
        Encoder encoder = encoder();
        if (encoder == null) {
            return 0.0;
        }
        double conversion = encoder.getConversion();
        double conversionOffset = encoder.getConversionOffset();
        double gearRatio = encoder.getGearRatio();
        double offset = encoder.getOffset();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double value = ((rawRotations - offset) * safeGearRatio) * safeConversion + conversionOffset;
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Converts a mechanism-space position into raw encoder rotations
     * using the encoder's gear ratio, conversion, and offsets.
     */
    default double mechanismPositionToEncoderRaw(double position) {
        Encoder encoder = encoder();
        if (encoder == null) {
            return 0.0;
        }
        double conversion = encoder.getConversion();
        double conversionOffset = encoder.getConversionOffset();
        double gearRatio = encoder.getGearRatio();
        double offset = encoder.getOffset();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double value = ((position - conversionOffset) / safeConversion) / safeGearRatio + offset;
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Converts a raw encoder velocity (rotations/sec) into mechanism-space units/sec.
     */
    default double encoderRawToMechanismVelocity(double rawRotationsPerSecond) {
        Encoder encoder = encoder();
        if (encoder == null) {
            return 0.0;
        }
        double conversion = encoder.getConversion();
        double gearRatio = encoder.getGearRatio();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double value = (rawRotationsPerSecond * safeGearRatio) * safeConversion;
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Converts a mechanism-space velocity (units/sec) into raw encoder rotations/sec.
     */
    default double mechanismVelocityToEncoderRaw(double velocity) {
        Encoder encoder = encoder();
        if (encoder == null) {
            return 0.0;
        }
        double conversion = encoder.getConversion();
        double gearRatio = encoder.getGearRatio();

        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double value = (velocity / safeConversion) / safeGearRatio;
        return Double.isFinite(value) ? value : 0.0;
    }

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

    /**
     * Returns a named PID controller registered in the mechanism config.
     */
    PIDController pid(String name);

    /**
     * Returns a named feedforward registered in the mechanism config.
     */
    SimpleMotorFeedforward feedforward(String name);

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
     * Converts a position value to rotations.
     */
    default double positionToRotations(double value, PositionUnit unit) {
        if (unit == null) {
            return value;
        }
        return switch (unit) {
            case ROTATIONS -> value;
            case RADIANS -> value / (2.0 * Math.PI);
            case DEGREES -> value / 360.0;
            case ENCODER_UNITS -> {
                Encoder encoder = mechanism().getEncoder();
                double conversion = encoder != null ? encoder.getConversion() : 0.0;
                if (!Double.isFinite(conversion) || conversion == 0.0) {
                    yield 0.0;
                }
                yield value / conversion;
            }
        };
    }

    /**
     * Converts a velocity value to rotations per second.
     */
    default double velocityToRotationsPerSecond(double value, VelocityUnit unit) {
        if (unit == null) {
            return value;
        }
        return switch (unit) {
            case ROTATIONS_PER_SEC -> value;
            case RAD_PER_SEC -> value / (2.0 * Math.PI);
            case DEG_PER_SEC -> value / 360.0;
            case ENCODER_UNITS_PER_SEC -> {
                Encoder encoder = mechanism().getEncoder();
                double conversion = encoder != null ? encoder.getConversion() : 0.0;
                if (!Double.isFinite(conversion) || conversion == 0.0) {
                    yield 0.0;
                }
                yield value / conversion;
            }
        };
    }

    /**
     * Converts a position value into the mechanism's output space.
     */
    default double positionToOutput(double value, PositionUnit unit) {
        double rotations = positionToRotations(value, unit);
        return toOutput(OutputType.POSITION, rotations);
    }

    /**
     * Converts a velocity value into the mechanism's output space.
     */
    default double velocityToOutput(double value, VelocityUnit unit) {
        double rps = velocityToRotationsPerSecond(value, unit);
        return toOutput(OutputType.VELOCITY, rps);
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
