package ca.frc6390.athena.mechanism.constraint;

import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.units.UnitConversions;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Factories and composition for mechanism constraints.
 */
public final class Constraints {
    private Constraints() {
    }

    public static Constraint<Double> range(Range range) {
        return new RangeConstraint(Objects.requireNonNull(range, "range"));
    }

    /**
     * Typed range limit. Both bounds must measure the same dimension (for example degrees); the
     * constraint is converted to the binding's unit when attached to a unit-typed control binding.
     *
     * @param minimum lower bound measure
     * @param maximum upper bound measure
     * @return range constraint in the bounds' dimension
     */
    public static Constraint<Double> range(Measure<?> minimum, Measure<?> maximum) {
        Objects.requireNonNull(minimum, "minimum");
        Objects.requireNonNull(maximum, "maximum");
        if (!UnitConversions.sameDimension(minimum.unit(), maximum.unit())) {
            throw new IllegalArgumentException("Range bounds must measure the same dimension.");
        }
        double low = minimum.baseUnitMagnitude();
        double high = maximum.baseUnitMagnitude();
        if (!(low <= high)) {
            throw new IllegalArgumentException("Range minimum must not exceed maximum.");
        }
        return new TypedRangeConstraint(minimum.unit(), low, high);
    }

    /**
     * Typed motion limits. {@code maximumVelocity} measures a rate of the control's unit (for example
     * degrees per second); {@code maximumAcceleration} measures a rate of that rate. Converted when
     * attached to a unit-typed control binding.
     *
     * @param maximumVelocity maximum velocity measure
     * @param maximumAcceleration maximum acceleration measure
     * @return motion constraint in the measures' dimension
     */
    public static Constraint<Double> motion(Measure<?> maximumVelocity, Measure<?> maximumAcceleration) {
        Objects.requireNonNull(maximumVelocity, "maximumVelocity");
        Objects.requireNonNull(maximumAcceleration, "maximumAcceleration");
        double velocity = maximumVelocity.baseUnitMagnitude();
        double acceleration = maximumAcceleration.baseUnitMagnitude();
        // Validates positivity exactly as MotionProfile does, in base units.
        new MotionProfile(velocity, acceleration);
        return new TypedMotionConstraint(maximumVelocity.unit(), maximumAcceleration.unit(), velocity, acceleration);
    }

    private record TypedRangeConstraint(Unit dimension, double minimumBase, double maximumBase)
            implements UnitConstraint {
        @Override
        public Constraint<Double> expressedIn(Unit unit) {
            if (!UnitConversions.sameDimension(dimension, unit)) {
                throw new IllegalArgumentException("Range limits measure " + dimension.name()
                        + " and cannot be applied to " + unit.name() + ".");
            }
            return range(Range.of(unit.ofBaseUnits(minimumBase), unit.ofBaseUnits(maximumBase)));
        }
    }

    private record TypedMotionConstraint(Unit velocityUnit, Unit accelerationUnit, double velocityBase,
            double accelerationBase) implements UnitConstraint {
        /**
         * Motion limits keep the same meaning for every mode: the first measure is the largest velocity (a rate of
         * the unit) and the second the largest acceleration. A velocity control clamps its target to the first and
         * slews the reference by the second, which is what the velocity loop enforces.
         */
        @Override
        public Constraint<Double> expressedIn(Unit unit, boolean velocityControl) {
            return expressedIn(unit);
        }

        @Override
        public Constraint<Double> expressedIn(Unit unit) {
            Unit perSecond = UnitConversions.perSecond(unit);
            if (!UnitConversions.sameDimension(velocityUnit, perSecond)
                    || !UnitConversions.sameDimension(accelerationUnit, UnitConversions.perSecond(perSecond))) {
                throw new IllegalArgumentException("Motion limits measure " + velocityUnit.name() + " and "
                        + accelerationUnit.name() + " and cannot be applied to " + unit.name() + ".");
            }
            double scale = UnitConversions.baseScale(unit);
            return new MotionConstraint(new MotionProfile(velocityBase / scale, accelerationBase / scale));
        }
    }

    private record RangeConstraint(Range range) implements Constraint<Double> {
        @Override
        public ConstraintResult<Double> evaluate(ConstraintContext<Double> context) {
            double requested = context.requested();
            double corrected = range.clamp(requested);
            return Double.compare(requested, corrected) == 0
                    ? new ConstraintResult.Allowed<>(requested)
                    : new ConstraintResult.Corrected<>(requested, corrected);
        }
    }

    private record MotionConstraint(MotionProfile profile) implements Constraint<Double> {
        @Override
        public ConstraintResult<Double> evaluate(ConstraintContext<Double> context) {
            return new ConstraintResult.Allowed<>(context.requested());
        }
    }

    /**
     * Clamps the final calculated voltage output to an inclusive range. The setpoint and the feedback are not
     * limited; use {@link #range(Measure, Measure)} for those. Applies only during output evaluation.
     *
     * @param minimum lower voltage bound
     * @param maximum upper voltage bound
     * @return output-stage voltage clamp
     * @throws IllegalArgumentException when a bound is not finite or the bounds are reversed
     */
    public static Constraint<Double> clamp(Voltage minimum, Voltage maximum) {
        double low = Objects.requireNonNull(minimum, "minimum").in(Units.Volts);
        double high = Objects.requireNonNull(maximum, "maximum").in(Units.Volts);
        if (!Double.isFinite(low) || !Double.isFinite(high) || high < low) {
            throw new IllegalArgumentException("Control clamp bounds must be finite and ordered.");
        }
        return new ClampConstraint(low, high);
    }

    private record ClampConstraint(double minimum, double maximum) implements Constraint<Double> {
        @Override
        public ConstraintResult<Double> evaluate(ConstraintContext<Double> context) {
            double requested = context.requested();
            double clamped = Math.max(minimum, Math.min(maximum, requested));
            return Double.compare(requested, clamped) == 0
                    ? new ConstraintResult.Allowed<>(requested)
                    : new ConstraintResult.Corrected<>(requested, clamped);
        }

        @Override
        public ConstraintStage stage() {
            return ConstraintStage.OUTPUT;
        }
    }

    /**
     * Limits the calculated voltage output to {@code +/- maximumAbsolute} at the output stage. This limits the
     * drive voltage (for example 12 V on a battery-powered motor), not the setpoint, so position and velocity
     * targets still reach the loop unchanged.
     *
     * @param maximumAbsolute largest output magnitude, non-negative and finite
     * @return output-stage voltage clamp
     * @throws IllegalArgumentException when the magnitude is negative or not finite
     */
    public static Constraint<Double> maxVoltage(Voltage maximumAbsolute) {
        double magnitude = Objects.requireNonNull(maximumAbsolute, "maximumAbsolute").in(Units.Volts);
        if (!Double.isFinite(magnitude) || magnitude < 0.0) {
            throw new IllegalArgumentException("Control clamp magnitude must be finite and non-negative.");
        }
        return new ClampConstraint(-magnitude, magnitude);
    }

    /**
     * Limits the calculated output to {@code +/- maximumAbsolute} at the output stage, for controls whose
     * output is not a motor voltage, such as a binding that drives a {@code RobotVelocityPool} angular channel
     * (radians per second) or a linear velocity channel (meters per second). The output is clamped in the
     * measure's SI base unit, which is the unit those channels take. Use {@link #maxVoltage(Voltage)} for
     * motor outputs.
     *
     * @param maximumAbsolute largest output magnitude, non-negative and finite
     * @return output-stage clamp
     * @throws IllegalArgumentException when the magnitude is negative or not finite
     */
    public static Constraint<Double> maxOutput(Measure<?> maximumAbsolute) {
        double magnitude = Objects.requireNonNull(maximumAbsolute, "maximumAbsolute").baseUnitMagnitude();
        if (!Double.isFinite(magnitude) || magnitude < 0.0) {
            throw new IllegalArgumentException("Control clamp magnitude must be finite and non-negative.");
        }
        return new ClampConstraint(-magnitude, magnitude);
    }

    public static Constraint<Double> lower(BooleanSupplier blocked) {
        Objects.requireNonNull(blocked, "blocked");
        return named("lower", context -> blocked.getAsBoolean() && context.requested() < context.current()
                ? new ConstraintResult.Rejected<>(context.requested())
                : new ConstraintResult.Allowed<>(context.requested()));
    }

    public static Constraint<Double> upper(BooleanSupplier blocked) {
        Objects.requireNonNull(blocked, "blocked");
        return named("upper", context -> blocked.getAsBoolean() && context.requested() > context.current()
                ? new ConstraintResult.Rejected<>(context.requested())
                : new ConstraintResult.Allowed<>(context.requested()));
    }

    public static Constraint<Double> require(BooleanSupplier allowed) {
        Objects.requireNonNull(allowed, "allowed");
        return named("require", context -> allowed.getAsBoolean()
                ? new ConstraintResult.Allowed<>(context.requested())
                : new ConstraintResult.Rejected<>(context.requested()));
    }

    public static <T> Constraint<T> require(Predicate<ConstraintContext<T>> allowed) {
        Objects.requireNonNull(allowed, "allowed");
        return named("require", context -> allowed.test(context)
                ? new ConstraintResult.Allowed<>(context.requested())
                : new ConstraintResult.Rejected<>(context.requested()));
    }

    /** Names a constraint for rejection reporting. Results are unchanged apart from the rejection's kind. */
    private static <T> Constraint<T> named(String kind, Constraint<T> delegate) {
        return new NamedConstraint<>(kind, delegate);
    }

    private record NamedConstraint<T>(String kind, Constraint<T> delegate) implements Constraint<T> {
        @Override
        public ConstraintResult<T> evaluate(ConstraintContext<T> context) {
            ConstraintResult<T> result = delegate.evaluate(context);
            return result instanceof ConstraintResult.Rejected<T> rejected
                    ? new ConstraintResult.Rejected<>(rejected.requested(), kind)
                    : result;
        }

        @Override
        public ConstraintStage stage() {
            return delegate.stage();
        }
    }

    public static <T> Constraint<T> correct(
            Predicate<ConstraintContext<T>> illegal,
            Function<ConstraintContext<T>, T> correction) {
        Objects.requireNonNull(illegal, "illegal");
        Objects.requireNonNull(correction, "correction");
        return context -> {
            if (!illegal.test(context)) {
                return new ConstraintResult.Allowed<>(context.requested());
            }
            return new ConstraintResult.Corrected<>(
                    context.requested(),
                    Objects.requireNonNull(correction.apply(context), "correction"));
        };
    }

    /** Replaces a requested value while a condition is active. */
    public static <T> Constraint<T> override(BooleanSupplier active, T value) {
        Objects.requireNonNull(active, "active");
        Objects.requireNonNull(value, "value");
        return context -> active.getAsBoolean()
                ? new ConstraintResult.Corrected<>(context.requested(), value)
                : new ConstraintResult.Allowed<>(context.requested());
    }

    @SafeVarargs
    public static <T> Constraint<T> all(Constraint<T>... constraints) {
        List<Constraint<T>> collected = new ArrayList<>();
        if (constraints != null) {
            for (Constraint<T> constraint : constraints) {
                if (constraint != null) {
                    collected.add(constraint);
                }
            }
        }
        List<Constraint<T>> copy = List.copyOf(collected);
        ConstraintStage stage = copy.isEmpty() ? ConstraintStage.TARGET : copy.get(0).stage();
        if (copy.stream().anyMatch(constraint -> constraint.stage() != stage)) {
            throw new IllegalArgumentException("A constraint group cannot mix target and output constraints.");
        }
        return new ConstraintGroup<>(copy, stage);
    }

    /** Returns whether a constraint collection contains the requested stage. */
    public static boolean hasStage(
            List<? extends Constraint<?>> constraints,
            ConstraintStage stage) {
        Objects.requireNonNull(stage, "stage");
        if (constraints == null) return false;
        for (int index = 0; index < constraints.size(); index++) {
            Constraint<?> constraint = constraints.get(index);
            if (constraint != null && constraint.stage() == stage) return true;
        }
        return false;
    }

    /**
     * Returns the intersection of declared position ranges, or an unbounded range. The result is
     * expressed in the unit of the first bounded range; other bounds are converted through base units.
     */
    public static Range positionRange(List<? extends Constraint<Double>> constraints) {
        Range bounded = intersectBounds(constraints);
        return bounded == null ? Range.ofUnit(edu.wpi.first.units.Units.Value, -Double.MAX_VALUE, Double.MAX_VALUE)
                : bounded;
    }

    /** Returns the intersection of the bounded ranges in these constraints, or {@code null} when none is bounded. */
    private static Range intersectBounds(List<? extends Constraint<?>> constraints) {
        Unit unit = null;
        double low = -Double.MAX_VALUE;
        double high = Double.MAX_VALUE;
        boolean bounded = false;
        if (constraints != null) {
            for (Constraint<?> constraint : constraints) {
                Range range = null;
                if (constraint instanceof RangeConstraint rangeConstraint) {
                    range = rangeConstraint.range();
                } else if (constraint instanceof ConstraintGroup<?> group) {
                    range = intersectBounds(group.constraints());
                }
                if (range == null) {
                    continue;
                }
                if (unit == null) {
                    unit = range.unit();
                }
                low = Math.max(low, range.minimum().baseUnitMagnitude());
                high = Math.min(high, range.maximum().baseUnitMagnitude());
                bounded = true;
            }
        }
        if (!bounded) {
            return null;
        }
        if (high < low) {
            throw new IllegalArgumentException("Control position constraints do not overlap.");
        }
        return Range.ofUnit(unit, unit.fromBaseUnits(low), unit.fromBaseUnits(high));
    }

    /** Returns the strictest declared motion limits, or {@code null} when absent. */
    public static MotionProfile motionProfile(List<? extends Constraint<Double>> constraints) {
        double velocity = Double.MAX_VALUE;
        double acceleration = Double.MAX_VALUE;
        boolean declared = false;
        if (constraints != null) {
            for (int index = 0; index < constraints.size(); index++) {
                Constraint<Double> constraint = constraints.get(index);
                if (constraint instanceof MotionConstraint motion) {
                    declared = true;
                    velocity = Math.min(velocity, motion.profile().maxVelocity());
                    acceleration = Math.min(acceleration, motion.profile().maxAcceleration());
                } else if (constraint instanceof ConstraintGroup<?> group) {
                    @SuppressWarnings("unchecked")
                    List<? extends Constraint<Double>> children =
                            (List<? extends Constraint<Double>>) (List<?>) group.constraints();
                    MotionProfile nested = motionProfile(children);
                    if (nested != null) {
                        declared = true;
                        velocity = Math.min(velocity, nested.maxVelocity());
                        acceleration = Math.min(acceleration, nested.maxAcceleration());
                    }
                }
            }
        }
        return declared ? new MotionProfile(velocity, acceleration) : null;
    }

    public static <T> ConstraintResult<T> evaluate(
            List<? extends Constraint<T>> constraints,
            ConstraintContext<T> context) {
        return evaluate(constraints, context, ConstraintStage.TARGET);
    }

    /** Evaluates constraints that govern the final calculated output. */
    public static <T> ConstraintResult<T> evaluateOutput(
            List<? extends Constraint<T>> constraints,
            ConstraintContext<T> context) {
        return evaluate(constraints, context, ConstraintStage.OUTPUT);
    }

    private static <T> ConstraintResult<T> evaluate(
            List<? extends Constraint<T>> constraints,
            ConstraintContext<T> context,
            ConstraintStage stage) {
        Objects.requireNonNull(context, "context");
        T requested = context.requested();
        boolean corrected = false;
        if (constraints != null) {
            for (int index = 0; index < constraints.size(); index++) {
                Constraint<T> constraint = constraints.get(index);
                if (constraint == null || constraint.stage() != stage) {
                    continue;
                }
                ConstraintResult<T> result = Objects.requireNonNull(
                        constraint.evaluate(context.requested(requested)),
                        "constraint result");
                if (!result.accepted()) {
                    return new ConstraintResult.Rejected<>(context.requested(), constraint.kind());
                }
                corrected |= result instanceof ConstraintResult.Corrected<?>;
                requested = result.value();
            }
        }
        return corrected
                ? new ConstraintResult.Corrected<>(context.requested(), requested)
                : new ConstraintResult.Allowed<>(requested);
    }

    private record ConstraintGroup<T>(List<Constraint<T>> constraints, ConstraintStage stage)
            implements Constraint<T> {
        private ConstraintGroup {
            constraints = List.copyOf(constraints);
            Objects.requireNonNull(stage, "stage");
        }

        @Override
        public ConstraintResult<T> evaluate(ConstraintContext<T> context) {
            return Constraints.evaluate(constraints, context, stage);
        }
    }
}
