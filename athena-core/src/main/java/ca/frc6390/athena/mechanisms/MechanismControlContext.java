package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.context.RobotScopedContext;
import ca.frc6390.athena.core.input.TypedInputContext;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
public interface MechanismControlContext<T extends Mechanism>
        extends TypedInputContext, RobotScopedContext {

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
        return mechanism().motors().device();
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
        return mechanism().encoder().device();
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
     * Current mechanism setpoint.
     */
    double setpoint();

    /**
     * Active state for stateful mechanisms, or {@code null} if none exists.
     */
    Enum<?> state();

    default RobotCore<?> robotCore() {
        RobotCore<?> core = mechanism().getRobotCore();
        return core != null ? core : RobotCore.getActiveInstance();
    }

    /**
     * Returns a named PID controller registered in the mechanism config.
     */
    PIDController pid(String name);

    /**
     * Returns a named profiled PID controller registered in the mechanism config, if present.
     */
    default ProfiledPIDController profiledPid(String name) {
        return null;
    }

    /**
     * Returns a named bang-bang profile registered in the mechanism config.
     */
    MechanismConfig.BangBangProfile bangBang(String name);

    /**
     * Returns a named feedforward registered in the mechanism config.
     */
    SimpleMotorFeedforward feedforward(String name);

    /**
     * Computes PID output for the named profile and converts it into the mechanism output space.
     *
     * <p>The PID profile declares the output units it was tuned in (percent or volts). This method
     * handles the conversion to the mechanism's configured output type.</p>
     */
    default double pidOut(String name, double measurement, double setpoint) {
        ProfiledPIDController profiled = profiledPid(name);
        double raw;
        if (profiled != null) {
            raw = profiled.calculate(measurement, setpoint);
        } else {
            PIDController controller = pid(name);
            if (controller == null) {
                return 0.0;
            }
            raw = controller.calculate(measurement, setpoint);
        }
        OutputType from = mechanism().getControlLoopPidOutputType(name);
        return toOutput(from, raw);
    }

    /**
     * Computes bang-bang output for the named profile and converts it into the mechanism output space.
     */
    default double bangBangOut(String name, double measurement, double setpoint) {
        MechanismConfig.BangBangProfile profile = bangBang(name);
        if (profile == null) {
            return 0.0;
        }
        OutputType from = profile.outputType() != null ? profile.outputType() : OutputType.PERCENT;
        double raw = Mechanism.calculateBangBangRaw(profile, measurement, setpoint);
        return toOutput(from, raw);
    }

    /**
     * Computes feedforward output for the named profile and converts it into the mechanism output space.
     *
     * <p>WPILib feedforward models output volts; profiles therefore use {@link OutputType#VOLTAGE}.</p>
     */
    default double feedforwardOut(String name, double velocity) {
        SimpleMotorFeedforward ff = feedforward(name);
        if (ff == null) {
            return 0.0;
        }
        double volts = ff.calculate(velocity);
        OutputType from = mechanism().getControlLoopFeedforwardOutputType(name);
        return toOutput(from, volts);
    }

    /**
     * Computes feedforward output using current and next velocity (better for acceleration terms).
     */
    default double feedforwardOut(String name, double currentVelocity, double nextVelocity) {
        SimpleMotorFeedforward ff = feedforward(name);
        if (ff == null) {
            return 0.0;
        }
        double volts = ff.calculateWithVelocities(currentVelocity, nextVelocity);
        OutputType from = mechanism().getControlLoopFeedforwardOutputType(name);
        return toOutput(from, volts);
    }

    default boolean usesVoltage() {
        return mechanism().useVoltage();
    }

    default OutputType outputType() {
        return mechanism().outputType();
    }

    default double batteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    /**
     * Converts a percent output (-1..1) into the mechanism's output space.
     * If the mechanism uses voltage, this returns volts. Otherwise it returns percent.
     */
    default double percentToOutput(double percent) {
        return OutputConversions.toMechanismOutput(
                outputType(),
                OutputType.PERCENT,
                percent,
                batteryVoltage());
    }

    /**
     * Converts volts into the mechanism's output space.
     * If the mechanism uses voltage, this returns volts. Otherwise it returns percent.
     */
    default double voltsToOutput(double volts) {
        return OutputConversions.toMechanismOutput(
                outputType(),
                OutputType.VOLTAGE,
                volts,
                batteryVoltage());
    }

    /**
     * Converts a value from the given output type into the mechanism's output space.
     */
    default double toOutput(OutputType type, double value) {
        return OutputConversions.toMechanismOutput(outputType(), type, value, batteryVoltage());
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
                Encoder encoder = mechanism().encoder().device();
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
                Encoder encoder = mechanism().encoder().device();
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
        if (outputType() == OutputType.POSITION || outputType() == OutputType.VELOCITY) {
            return output;
        }
        return OutputConversions.convert(outputType(), OutputType.PERCENT, output, batteryVoltage());
    }

    /**
     * Converts a mechanism output value into volts.
     */
    default double outputToVolts(double output) {
        if (outputType() == OutputType.POSITION || outputType() == OutputType.VELOCITY) {
            return output;
        }
        return OutputConversions.convert(outputType(), OutputType.VOLTAGE, output, batteryVoltage());
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
        mechanism().loops().disable(name);
    }

    default void enableControlLoop(String name) {
        mechanism().loops().enable(name);
    }

    default boolean isControlLoopEnabled(String name) {
        return mechanism().loops().enabled(name);
    }
    
}
