package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.runtime.control.ControlSink;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;

/**
 * Factories for control bindings. Every binding declares the WPILib unit its targets, feedback and
 * constraints are expressed in. Use {@code Units.Value} for a dimensionless binding.
 */
public final class Controls {
    private Controls() {
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

    /** Creates a dimensionless position control, for example one whose targets are a plain ratio. */
    public static ControlBinding<DimensionlessUnit> position(MotorDevice output, DimensionlessUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
    }

    /** Creates a rotary position control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<AngleUnit> position(ControlSink output, AngleUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
    }

    /** Creates a linear position control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<DistanceUnit> position(ControlSink output, DistanceUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
    }

    /** Creates a dimensionless position control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<DimensionlessUnit> position(ControlSink output, DimensionlessUnit unit) {
        return typed(ControlMode.POSITION, unit).output(output);
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

    /** Creates a dimensionless velocity control: targets are per-second rates of a plain ratio. */
    public static ControlBinding<DimensionlessUnit> velocity(MotorDevice output, DimensionlessUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    /** Creates a rotary velocity control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<AngleUnit> velocity(ControlSink output, AngleUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    /** Creates a linear velocity control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<DistanceUnit> velocity(ControlSink output, DistanceUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    /** Creates a dimensionless velocity control whose calculated result is applied to a scalar destination. */
    public static ControlBinding<DimensionlessUnit> velocity(ControlSink output, DimensionlessUnit unit) {
        return typed(ControlMode.VELOCITY, unit).output(output);
    }

    private static <U extends Unit> ControlBinding<U> typed(ControlMode mode, U unit) {
        java.util.Objects.requireNonNull(unit, "unit");
        return new ControlBinding<>(mode, null, 0, null, null, null, null, null, null, null, null,
                false, null, null, unit, null, null, null);
    }
}
