package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.runtime.control.ControlSink;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;

/**
 * Factories for control bindings.
 */
public final class Controls {
    private Controls() {
    }

    public static ControlBinding<?> position(MotorDevice output) {
        return of(ControlMode.POSITION).output(output);
    }

    /**
     * Creates a rotary position control whose targets, feedback and constraints are expressed in
     * {@code unit}. Encoder feedback must declare {@code measures(unit)}.
     */
    public static ControlBinding<AngleUnit> position(MotorDevice output, AngleUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
    }

    /**
     * Creates a linear position control whose targets, feedback and constraints are expressed in
     * {@code unit}. Encoder feedback must declare {@code measures(unit)}.
     */
    public static ControlBinding<DistanceUnit> position(MotorDevice output, DistanceUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
    }

    /** Creates a position control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<?> position(ControlSink output) {
        return of(ControlMode.POSITION).output(output);
    }

    /** Creates one position control that commands every supplied motor. */
    public static ControlBinding<?> position(MotorDevice output, MotorDevice... additionalOutputs) {
        return outputs(ControlMode.POSITION, output, additionalOutputs);
    }

    public static ControlBinding<?> velocity(MotorDevice output) {
        return of(ControlMode.VELOCITY).output(output);
    }

    /**
     * Creates a rotary velocity control. Targets are per-second rates of {@code unit}; encoder
     * feedback must declare {@code measures(unit)}.
     */
    public static ControlBinding<AngleUnit> velocity(MotorDevice output, AngleUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    /**
     * Creates a linear velocity control. Targets are per-second rates of {@code unit}; encoder
     * feedback must declare {@code measures(unit)}.
     */
    public static ControlBinding<DistanceUnit> velocity(MotorDevice output, DistanceUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    /** Creates a velocity control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<?> velocity(ControlSink output) {
        return of(ControlMode.VELOCITY).output(output);
    }

    /** Creates one velocity control that commands every supplied motor. */
    public static ControlBinding<?> velocity(MotorDevice output, MotorDevice... additionalOutputs) {
        return outputs(ControlMode.VELOCITY, output, additionalOutputs);
    }

    public static ControlBinding<?> of(ControlMode mode) {
        return new ControlBinding<>(mode, null, 0, null, null, null, null);
    }

    private static <U extends edu.wpi.first.units.Unit> ControlBinding<U> typed(ControlMode mode, U unit) {
        java.util.Objects.requireNonNull(unit, "unit");
        return new ControlBinding<>(mode, null, 0, null, null, null, null, null, null, null, null,
                false, null, null, unit, null, null, null);
    }

    private static ControlBinding<?> outputs(
            ControlMode mode,
            MotorDevice output,
            MotorDevice... additionalOutputs) {
        return of(mode).output(output).followers(additionalOutputs);
    }
}
