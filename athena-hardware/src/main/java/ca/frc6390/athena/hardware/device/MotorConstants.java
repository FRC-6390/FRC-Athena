package ca.frc6390.athena.hardware.device;

import ca.frc6390.athena.api.hardware.MotorKind;
import java.util.Optional;

/**
 * Electrical and mechanical constants of one physical motor at nominal voltage.
 *
 * <p>The values are derived from the stall and free-run points of a motor's published
 * characteristic: {@code R = Vnom / stallCurrent}, {@code Kt = stallTorque / stallCurrent} and the
 * speed constant {@code Kv = freeSpeed / (Vnom - R * freeCurrent)}. Built-in constants are
 * datasheet-level approximations. They are good enough for stall gating and motor-effort estimates,
 * not for exact simulation, and teams can supply their own with {@link #of(double, double, double, double)}.</p>
 *
 * @param stallTorqueNm stall torque in newton-metres
 * @param stallCurrentAmps stall current in amps
 * @param freeCurrentAmps no-load current in amps
 * @param freeSpeedRadPerSec no-load speed in radians per second
 * @param nominalVoltage voltage the stall and free points were measured at
 * @param resistanceOhms winding resistance
 * @param torqueConstant torque per amp, newton-metres per amp
 * @param speedConstant no-load speed per volt of back-EMF, radians per second per volt
 */
public record MotorConstants(
        double stallTorqueNm,
        double stallCurrentAmps,
        double freeCurrentAmps,
        double freeSpeedRadPerSec,
        double nominalVoltage,
        double resistanceOhms,
        double torqueConstant,
        double speedConstant) {

    /** Nominal battery voltage used by the published characteristics. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    public MotorConstants {
        requirePositive(stallTorqueNm, "stall torque");
        requirePositive(stallCurrentAmps, "stall current");
        requireNonNegative(freeCurrentAmps, "free current");
        requirePositive(freeSpeedRadPerSec, "free speed");
        requirePositive(nominalVoltage, "nominal voltage");
        requirePositive(resistanceOhms, "resistance");
        requirePositive(torqueConstant, "torque constant");
        requirePositive(speedConstant, "speed constant");
    }

    /**
     * Derives the full constant set from the published stall and free-run points at 12 V.
     *
     * @param stallTorqueNm stall torque in newton-metres
     * @param stallCurrentAmps stall current in amps
     * @param freeCurrentAmps no-load current in amps
     * @param freeSpeedRpm no-load speed in revolutions per minute
     * @return constants for the motor
     */
    public static MotorConstants of(
            double stallTorqueNm,
            double stallCurrentAmps,
            double freeCurrentAmps,
            double freeSpeedRpm) {
        requirePositive(stallCurrentAmps, "stall current");
        double resistance = NOMINAL_VOLTAGE / stallCurrentAmps;
        double freeSpeed = freeSpeedRpm * 2.0 * Math.PI / 60.0;
        double backEmfAtFree = NOMINAL_VOLTAGE - resistance * freeCurrentAmps;
        if (backEmfAtFree <= 0.0) {
            throw new IllegalArgumentException("Free current is too high for the stall current.");
        }
        return new MotorConstants(
                stallTorqueNm,
                stallCurrentAmps,
                freeCurrentAmps,
                freeSpeed,
                NOMINAL_VOLTAGE,
                resistance,
                stallTorqueNm / stallCurrentAmps,
                freeSpeed / backEmfAtFree);
    }

    /**
     * Returns the built-in constants for a known physical motor model.
     *
     * @param kind motor kind, may be null
     * @return constants, or empty when the model has no built-in constants
     */
    public static Optional<MotorConstants> forKind(MotorKind kind) {
        return MotorSpec.forKind(kind).map(MotorSpec::constants);
    }

    /**
     * Returns the no-load speed the applied voltage would produce, ignoring load.
     *
     * <p>The winding drop uses the free-run current rather than the measured current. At a true
     * stall the back-EMF is zero, so a drop computed from the measured current would predict zero
     * speed and hide the stall.</p>
     *
     * @param appliedVolts voltage applied to the motor
     * @return expected unloaded speed in radians per second, may be negative for very low voltage
     */
    public double expectedUnloadedSpeedRadPerSec(double appliedVolts) {
        return (appliedVolts - freeCurrentAmps * resistanceOhms) * speedConstant;
    }

    /** Returns the motor-side torque produced by a current, in newton-metres. */
    public double torqueAt(double currentAmps) {
        return currentAmps * torqueConstant;
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
}
