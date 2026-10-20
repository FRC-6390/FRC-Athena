package ca.frc6390.athena.hardware.device;

import ca.frc6390.athena.api.hardware.MotorKind;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

/**
 * Electrical and mechanical constants of one physical motor at nominal voltage.
 *
 * <p>The values are derived from the stall and free-run points of a motor's published
 * characteristic: {@code R = Vnom / stallCurrent}, {@code Kt = stallTorque / stallCurrent} and the
 * speed constant {@code Kv = freeSpeed / (Vnom - R * freeCurrent)}. Built-in constants are
 * datasheet-level approximations. They are good enough for stall gating and motor-effort estimates,
 * not for exact simulation, and teams can supply their own with {@link #of(Torque, Current, Current, AngularVelocity)}.</p>
 *
 * <p>Torque constant and speed constant have no WPILib unit (newton-metres per amp, radians per second
 * per volt). Their accessors are per-cycle primitives in those units, documented below.</p>
 *
 * @param stallTorque stall torque
 * @param stallCurrent stall current
 * @param freeCurrent no-load current
 * @param freeSpeed no-load speed
 * @param nominalVoltage voltage the stall and free points were measured at
 * @param resistance winding resistance
 */
public record MotorConstants(
        Torque stallTorque,
        Current stallCurrent,
        Current freeCurrent,
        AngularVelocity freeSpeed,
        Voltage nominalVoltage,
        Resistance resistance) {

    /** Nominal battery voltage used by the published characteristics. */
    public static final Voltage NOMINAL_VOLTAGE = Units.Volts.of(12.0);

    public MotorConstants {
        requirePositive(stallTorque.in(Units.NewtonMeters), "stall torque");
        requirePositive(stallCurrent.in(Units.Amps), "stall current");
        requireNonNegative(freeCurrent.in(Units.Amps), "free current");
        requirePositive(freeSpeed.in(Units.RadiansPerSecond), "free speed");
        requirePositive(nominalVoltage.in(Units.Volts), "nominal voltage");
        requirePositive(resistance.in(Units.Ohms), "resistance");
    }

    /**
     * Derives the full constant set from the published stall and free-run points at 12 V. The
     * resistance is derived from the stall point and the nominal voltage.
     *
     * @param stallTorque stall torque
     * @param stallCurrent stall current
     * @param freeCurrent no-load current
     * @param freeSpeed no-load speed
     * @return constants for the motor
     */
    public static MotorConstants of(Torque stallTorque, Current stallCurrent, Current freeCurrent,
            AngularVelocity freeSpeed) {
        requirePositive(stallCurrent.in(Units.Amps), "stall current");
        double nominalVolts = NOMINAL_VOLTAGE.in(Units.Volts);
        double resistanceOhms = nominalVolts / stallCurrent.in(Units.Amps);
        double backEmfAtFree = nominalVolts - resistanceOhms * freeCurrent.in(Units.Amps);
        if (backEmfAtFree <= 0.0) {
            throw new IllegalArgumentException("Free current is too high for the stall current.");
        }
        return new MotorConstants(
                stallTorque,
                stallCurrent,
                freeCurrent,
                freeSpeed,
                NOMINAL_VOLTAGE,
                Units.Ohms.of(resistanceOhms));
    }

    /**
     * Returns the built-in constants for a known physical motor model.
     *
     * @param kind motor kind, may be null
     * @return constants, or empty when the model has no built-in constants
     */
    public static Optional<MotorConstants> forKind(MotorKind kind) {
        return MotorSpec.constantsForKind(kind);
    }

    /**
     * Returns the motor-side torque produced by a current.
     *
     * @param current motor current
     * @return torque, {@code current * Kt}
     */
    public Torque torqueAt(Current current) {
        return Units.NewtonMeters.of(current.in(Units.Amps) * torqueConstantNmPerAmp());
    }

    /**
     * Returns the no-load speed the applied voltage would produce, ignoring load.
     *
     * <p>The winding drop uses the free-run current rather than the measured current. At a true
     * stall the back-EMF is zero, so a drop computed from the measured current would predict zero
     * speed and hide the stall.</p>
     *
     * @param appliedVoltage voltage applied to the motor; may be negative for very low voltage
     * @return expected unloaded speed
     */
    public AngularVelocity expectedUnloadedSpeed(Voltage appliedVoltage) {
        return Units.RadiansPerSecond.of(expectedUnloadedSpeedRadPerSec(appliedVoltage.in(Units.Volts)));
    }

    /**
     * Per-cycle primitive for {@link #expectedUnloadedSpeed(Voltage)}, in radians per second and volts.
     * Athena's hot paths call this to avoid allocating measures; robot code should use the typed method.
     *
     * @param appliedVolts applied voltage in volts
     * @return expected unloaded speed in radians per second
     */
    double expectedUnloadedSpeedRadPerSec(double appliedVolts) {
        return (appliedVolts - freeCurrent.in(Units.Amps) * resistance.in(Units.Ohms))
                * speedConstantRadPerSecPerVolt();
    }

    /**
     * Per-cycle primitive: torque constant {@code Kt = stallTorque / stallCurrent}, newton-metres per amp.
     * No WPILib unit exists for this ratio.
     *
     * @return torque constant in newton-metres per amp
     */
    double torqueConstantNmPerAmp() {
        return stallTorque.in(Units.NewtonMeters) / stallCurrent.in(Units.Amps);
    }

    /**
     * Per-cycle primitive: speed constant {@code Kv}, radians per second per volt of back-EMF.
     * No WPILib unit exists for this ratio.
     *
     * @return speed constant in radians per second per volt
     */
    double speedConstantRadPerSecPerVolt() {
        double backEmfAtFree = nominalVoltage.in(Units.Volts)
                - resistance.in(Units.Ohms) * freeCurrent.in(Units.Amps);
        return freeSpeed.in(Units.RadiansPerSecond) / backEmfAtFree;
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException("Motor " + name + " must be positive and finite.");
        }
    }

    private static void requireNonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("Motor " + name + " must be non-negative and finite.");
        }
    }

    /** Returns an internal, primitive view of this value. Not for robot code. */
    public Raw raw() {
        return new Raw(this);
    }

    /**
     * Internal, not for robot code: primitive constants for Athena's hot paths in the units the names state.
     * Robot code uses the typed accessors (expectedUnloadedSpeed(Voltage), ...).
     */
    public static final class Raw {
        private final MotorConstants owner;

        private Raw(MotorConstants owner) {
            this.owner = owner;
        }

        /** Internal: see MotorConstants#expectedUnloadedSpeedRadPerSec. */
        public double expectedUnloadedSpeedRadPerSec(double appliedVolts) {
            return owner.expectedUnloadedSpeedRadPerSec(appliedVolts);
        }

        /** Internal: see MotorConstants#torqueConstantNmPerAmp. */
        public double torqueConstantNmPerAmp() {
            return owner.torqueConstantNmPerAmp();
        }

        /** Internal: see MotorConstants#speedConstantRadPerSecPerVolt. */
        public double speedConstantRadPerSecPerVolt() {
            return owner.speedConstantRadPerSecPerVolt();
        }
    }
}
