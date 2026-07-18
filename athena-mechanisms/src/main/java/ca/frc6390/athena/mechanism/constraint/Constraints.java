package ca.frc6390.athena.mechanism.constraint;

import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
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

    /** Declares maximum reference velocity and acceleration for a control. */
    public static Constraint<Double> motion(double maximumVelocity, double maximumAcceleration) {
        return new MotionConstraint(new MotionProfile(maximumVelocity, maximumAcceleration));
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

    /** Clamps the final calculated control output to an inclusive range. */
    public static Constraint<Double> clamp(double minimum, double maximum) {
        if (!Double.isFinite(minimum) || !Double.isFinite(maximum) || maximum < minimum) {
            throw new IllegalArgumentException("Control clamp bounds must be finite and ordered.");
        }
        return new ClampConstraint(minimum, maximum);
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

    /** Clamps the final calculated control output symmetrically around zero. */
    public static Constraint<Double> clamp(double maximumAbsolute) {
        if (!Double.isFinite(maximumAbsolute) || maximumAbsolute < 0.0) {
            throw new IllegalArgumentException("Control clamp magnitude must be finite and non-negative.");
        }
        return clamp(-maximumAbsolute, maximumAbsolute);
    }

    public static Constraint<Double> lower(BooleanSupplier blocked) {
        Objects.requireNonNull(blocked, "blocked");
        return context -> blocked.getAsBoolean() && context.requested() < context.current()
                ? new ConstraintResult.Rejected<>(context.requested())
                : new ConstraintResult.Allowed<>(context.requested());
    }

    public static Constraint<Double> upper(BooleanSupplier blocked) {
        Objects.requireNonNull(blocked, "blocked");
        return context -> blocked.getAsBoolean() && context.requested() > context.current()
                ? new ConstraintResult.Rejected<>(context.requested())
                : new ConstraintResult.Allowed<>(context.requested());
    }

    public static Constraint<Double> require(BooleanSupplier allowed) {
        Objects.requireNonNull(allowed, "allowed");
        return context -> allowed.getAsBoolean()
                ? new ConstraintResult.Allowed<>(context.requested())
                : new ConstraintResult.Rejected<>(context.requested());
    }

    public static <T> Constraint<T> require(Predicate<ConstraintContext<T>> allowed) {
        Objects.requireNonNull(allowed, "allowed");
        return context -> allowed.test(context)
                ? new ConstraintResult.Allowed<>(context.requested())
                : new ConstraintResult.Rejected<>(context.requested());
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
        return constraints != null && constraints.stream()
                .filter(Objects::nonNull)
                .anyMatch(constraint -> constraint.stage() == stage);
    }

    /** Returns the intersection of declared position ranges, or an unbounded range. */
    public static Range positionRange(List<? extends Constraint<Double>> constraints) {
        double minimum = -Double.MAX_VALUE;
        double maximum = Double.MAX_VALUE;
        if (constraints != null) {
            for (Constraint<Double> constraint : constraints) {
                if (constraint instanceof RangeConstraint bounded) {
                    minimum = Math.max(minimum, bounded.range().minimum());
                    maximum = Math.min(maximum, bounded.range().maximum());
                } else if (constraint instanceof ConstraintGroup<?> group) {
                    @SuppressWarnings("unchecked")
                    List<? extends Constraint<Double>> children =
                            (List<? extends Constraint<Double>>) (List<?>) group.constraints();
                    Range nested = positionRange(children);
                    minimum = Math.max(minimum, nested.minimum());
                    maximum = Math.min(maximum, nested.maximum());
                }
            }
        }
        if (maximum < minimum) {
            throw new IllegalArgumentException("Control position constraints do not overlap.");
        }
        return Range.of(minimum, maximum);
    }

    /** Returns the strictest declared motion limits, or {@code null} when absent. */
    public static MotionProfile motionProfile(List<? extends Constraint<Double>> constraints) {
        double velocity = Double.MAX_VALUE;
        double acceleration = Double.MAX_VALUE;
        boolean declared = false;
        if (constraints != null) {
            for (Constraint<Double> constraint : constraints) {
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
            for (Constraint<T> constraint : constraints) {
                if (constraint == null || constraint.stage() != stage) {
                    continue;
                }
                ConstraintResult<T> result = Objects.requireNonNull(
                        constraint.evaluate(context.requested(requested)),
                        "constraint result");
                if (!result.accepted()) {
                    return new ConstraintResult.Rejected<>(context.requested());
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
