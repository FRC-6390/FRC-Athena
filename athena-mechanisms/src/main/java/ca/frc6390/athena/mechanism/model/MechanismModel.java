package ca.frc6390.athena.mechanism.model;

import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.Units;
import ca.frc6390.athena.hardware.sim.SimDynamicsSource;
import ca.frc6390.athena.hardware.sim.SimPlant;
import ca.frc6390.athena.hardware.units.MotionKind;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Identified parameters of one mechanism. It is an immutable value that can be copied, compared,
 * stored, and passed to simulation.
 *
 * <p>Units follow {@link Identification}: rotary positions in rotations, linear in meters, and
 * accelerations and velocities in the same base units per second. Unknown values are NaN
 * ({@link #backlash()}, {@link #latencySeconds()}) or 0.</p>
 *
 * @param motionKind rotary or linear, from the control unit
 * @param staticFrictionVolts breakaway friction voltage, sign-independent
 * @param kineticFrictionVolts kinetic kS
 * @param kV velocity gain, volts per base unit per second
 * @param kA acceleration gain, volts per base unit per second squared
 * @param kG gravity amplitude in volts (linear: constant)
 * @param gravityOffsetRadians phase of the gravity term, 0 for linear
 * @param friction kinetic friction offset versus position
 * @param backlash lost motion per reversal in base units, NaN when unknown
 * @param latencySeconds command-to-motion latency, NaN when unknown
 * @param payloadDelta change of kG from a confident payload estimate, 0 otherwise
 * @param motorTorqueConstant per-motor torque constant in N m per A, 0 when unknown
 * @param motorResistance per-motor winding resistance in ohms, 0 when unknown
 * @param motorCount motors driving the mechanism
 * @param motorKinds physical motor model keys
 * @param gearRatio motor rotations per mechanism rotation
 * @param standardErrors standard error per parameter, keyed by parameter name
 * @param rSquared coefficient of determination of the feedforward fit
 * @param sampleCount samples used to produce this model
 */
public record MechanismModel(
        MotionKind motionKind,
        double staticFrictionVolts,
        double kineticFrictionVolts,
        double kV,
        double kA,
        double kG,
        double gravityOffsetRadians,
        FrictionTable friction,
        double backlash,
        double latencySeconds,
        double payloadDelta,
        double motorTorqueConstant,
        double motorResistance,
        int motorCount,
        List<String> motorKinds,
        double gearRatio,
        Map<String, Double> standardErrors,
        double rSquared,
        int sampleCount) implements SimDynamicsSource {

    private static final double MOVING_EPSILON = 1.0e-9;

    public MechanismModel {
        Objects.requireNonNull(motionKind, "motionKind");
        friction = friction == null ? FrictionTable.EMPTY : friction;
        motorKinds = motorKinds == null ? List.of() : List.copyOf(motorKinds);
        if (!(gearRatio > 0.0) || !Double.isFinite(gearRatio)) {
            throw new IllegalArgumentException("Gear ratio must be positive and finite.");
        }
        if (motorCount < 0 || sampleCount < 0) {
            throw new IllegalArgumentException("Motor and sample counts cannot be negative.");
        }
        standardErrors = standardErrors == null ? Map.of() : Map.copyOf(standardErrors);
    }

    /** Returns a model with every parameter zero and unknown values NaN. */
    public static MechanismModel empty(MotionKind kind) {
        return new MechanismModel(kind, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FrictionTable.EMPTY,
                Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0, Map.of(), 0.0, 0);
    }

    /**
     * Returns the gravity term at a position, including any payload delta.
     *
     * @param position position in base units (rotations for rotary)
     * @return volts
     */
    public double gravityVolts(double position) {
        return Identification.gravityVolts(Gravity.of(motionKind), kG + payloadDelta, gravityOffsetRadians, position);
    }

    /**
     * Predicts the feedforward voltage for a motion state.
     *
     * @param position position in base units
     * @param velocity velocity in base units per second
     * @param acceleration acceleration in base units per second squared
     * @param directionHint sign of the desired motion, used only when the mechanism is at rest
     * @return volts
     */
    public double feedforwardVolts(double position, double velocity, double acceleration, double directionHint) {
        double sign;
        double frictionMagnitude;
        if (Math.abs(velocity) > MOVING_EPSILON) {
            sign = Math.signum(velocity);
            frictionMagnitude = kineticFrictionVolts;
        } else {
            sign = Math.signum(directionHint);
            frictionMagnitude = staticFrictionVolts;
        }
        return sign * (frictionMagnitude + friction.at(position))
                + kV * velocity
                + kA * acceleration
                + gravityVolts(position);
    }

    /**
     * Mechanism-side moment of inertia in kg m^2, from kA and the motor constants:
     * {@code J = (kA / 2 pi) * motors * gear * Kt / R}. Returns NaN for linear mechanisms or when the
     * motor constants are unknown.
     */
    public double inertia() {
        if (motionKind != MotionKind.ROTARY || kA <= 0.0 || motorTorqueConstant <= 0.0
                || motorResistance <= 0.0 || motorCount <= 0) {
            return Double.NaN;
        }
        return kA / (2.0 * Math.PI) * motorCount * gearRatio * motorTorqueConstant / motorResistance;
    }

    /** Returns {@link #inertia()} as an optional value for simulation. */
    @Override
    public Optional<MomentOfInertia> momentOfInertia() {
        double inertia = inertia();
        return Double.isFinite(inertia) ? Optional.of(edu.wpi.first.units.Units.KilogramSquareMeters.of(inertia)) : Optional.empty();
    }

    /**
     * Returns the identified plant for simulation: static and kinetic friction, kV, kA, gravity (including any
     * payload delta) and the motor constants that convert volts to mechanism torque or force. Empty until kA
     * and the motor constants are known.
     */
    @Override
    public Optional<SimPlant> plant() {
        if (!(kA > 0.0) || !(motorTorqueConstant > 0.0) || !(motorResistance > 0.0) || motorCount <= 0) {
            return Optional.empty();
        }
        return Optional.of(new SimPlant(
                motionKind == MotionKind.ROTARY,
                staticFrictionVolts,
                kineticFrictionVolts,
                kV,
                kA,
                kG + payloadDelta,
                gravityOffsetRadians,
                motorTorqueConstant,
                motorResistance,
                motorCount,
                gearRatio));
    }

    /** Returns the scalar parameters as a name-to-value map, for telemetry and storage. */
    public Map<String, Double> values() {
        Map<String, Double> values = new LinkedHashMap<>();
        values.put("staticFrictionVolts", staticFrictionVolts);
        values.put("kineticFrictionVolts", kineticFrictionVolts);
        values.put("kV", kV);
        values.put("kA", kA);
        values.put("kG", kG);
        values.put("gravityOffsetRadians", gravityOffsetRadians);
        values.put("backlash", backlash);
        values.put("latencySeconds", latencySeconds);
        values.put("payloadDelta", payloadDelta);
        values.put("rSquared", rSquared);
        values.put("sampleCount", (double) sampleCount);
        return values;
    }

    /** Returns a copy with a different payload delta. */
    public MechanismModel withPayloadDelta(double delta) {
        return new MechanismModel(motionKind, staticFrictionVolts, kineticFrictionVolts, kV, kA, kG,
                gravityOffsetRadians, friction, backlash, latencySeconds, delta, motorTorqueConstant,
                motorResistance, motorCount, motorKinds, gearRatio, standardErrors, rSquared, sampleCount);
    }

    /** Returns a copy with a different motor description. */
    public MechanismModel withMotors(double torqueConstant, double resistance, int count,
            List<String> kinds, double gear) {
        return new MechanismModel(motionKind, staticFrictionVolts, kineticFrictionVolts, kV, kA, kG,
                gravityOffsetRadians, friction, backlash, latencySeconds, payloadDelta, torqueConstant,
                resistance, count, new ArrayList<>(kinds), gear, standardErrors, rSquared, sampleCount);
    }
}
