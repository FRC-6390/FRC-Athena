package ca.frc6390.athena.mechanism.effort;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.ForceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Declaration-time conversion of effort measures into per-motor current limits (B5).
 *
 * <p>For a binding with {@code N} motors, gearing {@code g} (motor turns per mechanism turn) and
 * per-motor torque constant {@code Kt_i}, a mechanism torque {@code T} maps to a per-motor current of
 * {@code I_i = T / (N * g * Kt_i)}. A linear force {@code F} is first converted to torque with the drum
 * radius {@code r}: {@code T = F * r}. Current is already per motor. Hardware followers mirror the
 * leader's command, so every motor of the binding (leader and followers) is counted in {@code N}.</p>
 */
public final class EffortUnits {
    /** Nominal bus voltage used to express percent outputs as volts when a limit applies. */
    public static final double NOMINAL_VOLTS = 12.0;

    /** Physical kind of an effort measure. */
    public enum Kind {
        /** Newton metres at the mechanism. */
        TORQUE,
        /** Newtons at the mechanism. */
        FORCE,
        /** Amps per motor. */
        CURRENT
    }

    private EffortUnits() {
    }

    /**
     * Validates the effort measure's kind against the motion kind of a control.
     *
     * @param measure effort measure
     * @param motion motion kind of the binding
     * @return the effort kind
     * @throws IllegalArgumentException when the kind does not suit the binding
     */
    public static Kind kind(Measure<?> measure, MotionKind motion) {
        Objects.requireNonNull(measure, "measure");
        Objects.requireNonNull(motion, "motion");
        Unit unit = measure.unit();
        Kind kind = unit instanceof TorqueUnit ? Kind.TORQUE
                : unit instanceof ForceUnit ? Kind.FORCE
                : unit instanceof CurrentUnit ? Kind.CURRENT
                : null;
        if (kind == null) {
            throw new IllegalArgumentException("Effort must be a torque, force or current measure, not "
                    + unit.name() + ".");
        }
        boolean accepted;
        String message;
        switch (motion) {
            case ROTARY:
                accepted = kind != Kind.FORCE;
                message = "Rotary control accepts Torque (N m) or Current (A) effort, not " + unit.name() + ".";
                break;
            case LINEAR:
                accepted = kind != Kind.TORQUE;
                message = "Linear control accepts Force (N) or Current (A) effort, not " + unit.name() + ".";
                break;
            default:
                accepted = kind == Kind.CURRENT;
                message = "Untyped control accepts Current (A) effort only, not " + unit.name()
                        + ". Declare a unit (Controls.position(motor, unit)) for torque or force.";
                break;
        }
        if (!accepted) {
            throw new IllegalArgumentException(message);
        }
        return kind;
    }

    /**
     * Converts a declared effort into per-motor current limits, one per motor in list order.
     *
     * @param effort effort measure, already validated with {@link #kind(Measure, MotionKind)}
     * @param kind kind of {@code effort}
     * @param motors motors of the binding (leader first, then followers); must not be empty
     * @param gearing motor turns per mechanism turn, NaN when unknown
     * @param radius linear radius as a distance measure, or null
     * @return amps per motor
     * @throws IllegalArgumentException when torque or force lacks gearing, radius or motor constants
     */
    public static double[] perMotorAmps(Measure<?> effort, Kind kind, List<MotorDevice> motors,
            double gearing, Measure<?> radius) {
        Objects.requireNonNull(effort, "effort");
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(motors, "motors");
        if (motors.isEmpty()) {
            throw new IllegalArgumentException("Effort needs at least one motor.");
        }
        double value = unitOf(kind).fromBaseUnits(effort.baseUnitMagnitude());
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("Effort must be finite and non-negative.");
        }
        double[] amps = new double[motors.size()];
        if (kind == Kind.CURRENT) {
            Arrays.fill(amps, value);
            return amps;
        }
        String name = kind == Kind.TORQUE ? "Torque" : "Force";
        if (!(gearing > 0.0) || !Double.isFinite(gearing)) {
            throw new IllegalArgumentException(name + " effort requires known gearing — use Amps, "
                    + "or declare Effort.gearing(...).");
        }
        double torque = value;
        if (kind == Kind.FORCE) {
            if (radius == null) {
                throw new IllegalArgumentException("Force effort requires the drum radius — use Amps, "
                        + "or declare Effort.radius(...).");
            }
            torque = value * Units.Meters.fromBaseUnits(radius.baseUnitMagnitude());
        }
        double count = motors.size();
        for (int i = 0; i < amps.length; i++) {
            MotorDevice motor = motors.get(i);
            Optional<MotorConstants> constants = constants(motor);
            if (constants.isEmpty()) {
                throw new IllegalArgumentException(name + " effort needs torque constants for motor "
                        + motor.defaultName() + " — use Amps for this motor kind.");
            }
            amps[i] = torque / (count * gearing * constants.get().raw().torqueConstantNmPerAmp());
        }
        return amps;
    }

    /**
     * Returns the mechanism torque (N m) that per-motor currents produce, using each motor's torque
     * constant and the gearing. Used for load-response thresholds and telemetry.
     *
     * @param amps per-motor current, one per motor
     * @param motors motors, same order
     * @param gearing motor turns per mechanism turn
     * @return mechanism torque in N m, NaN when a motor's constants are unknown
     */
    public static double mechanismTorque(double[] amps, List<MotorDevice> motors, double gearing) {
        double torque = 0.0;
        for (int i = 0; i < amps.length; i++) {
            Optional<MotorConstants> constants = constants(motors.get(i));
            if (constants.isEmpty() || !Double.isFinite(amps[i])) {
                return Double.NaN;
            }
            torque += amps[i] * constants.get().raw().torqueConstantNmPerAmp() * gearing;
        }
        return torque;
    }

    /** Returns the built-in constants of a motor, or empty for motor kinds without them. */
    public static Optional<MotorConstants> constants(MotorDevice motor) {
        MotorKind kind = motor.kind();
        return MotorConstants.forKind(kind);
    }

    private static Unit unitOf(Kind kind) {
        switch (kind) {
            case TORQUE:
                return Units.NewtonMeters;
            case FORCE:
                return Units.Newtons;
            default:
                return Units.Amps;
        }
    }
}
