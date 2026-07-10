package ca.frc6390.athena.mechanism.constraint;

import ca.frc6390.athena.hardware.device.Range;
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
        Objects.requireNonNull(range, "range");
        return context -> {
            double requested = context.requested();
            double corrected = range.clamp(requested);
            return Double.compare(requested, corrected) == 0
                    ? new ConstraintResult.Allowed<>(requested)
                    : new ConstraintResult.Corrected<>(requested, corrected);
        };
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
        return context -> evaluate(copy, context);
    }

    public static <T> ConstraintResult<T> evaluate(
            List<? extends Constraint<T>> constraints,
            ConstraintContext<T> context) {
        Objects.requireNonNull(context, "context");
        T requested = context.requested();
        boolean corrected = false;
        if (constraints != null) {
            for (Constraint<T> constraint : constraints) {
                if (constraint == null) {
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
}
