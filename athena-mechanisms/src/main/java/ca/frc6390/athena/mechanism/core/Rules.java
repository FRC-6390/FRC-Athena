package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RangeRef;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Factories for mechanism rules.
 */
public final class Rules {
    private Rules() {
    }

    public static Builder when(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return when(ctx -> condition.getAsBoolean());
    }

    public static Builder when(Predicate<RuleContext> condition) {
        return new Builder(Objects.requireNonNull(condition, "condition"));
    }

    public static RangeBuilder range(RangeRef range) {
        return new RangeBuilder(Objects.requireNonNull(range, "range"));
    }

    public static final class Builder {
        private final Predicate<RuleContext> condition;
        private OutputTarget target = OutputTarget.any();

        private Builder(Predicate<RuleContext> condition) {
            this.condition = condition;
        }

        public Builder appliesTo(OutputTarget target) {
            this.target = Objects.requireNonNull(target, "target");
            return this;
        }

        public RuleRef warn() {
            return result(ctx -> RuleResult.warn());
        }

        public RuleRef warn(String reason) {
            return result(ctx -> RuleResult.warn(reason));
        }

        public RuleRef block() {
            return result(ctx -> RuleResult.block());
        }

        public RuleRef block(String reason) {
            return result(ctx -> RuleResult.block(reason));
        }

        public RuleRef clamp(double value) {
            return result(ctx -> RuleResult.clamp(value));
        }

        public RuleRef clamp(double value, String reason) {
            return result(ctx -> RuleResult.clamp(value, reason));
        }

        public RuleRef result(Function<RuleContext, RuleResult> result) {
            return new RuleRef(condition, target, Objects.requireNonNull(result, "result"), "");
        }
    }

    public static final class RangeBuilder {
        private final RangeRef range;
        private OutputTarget target = OutputTarget.any().position();

        private RangeBuilder(RangeRef range) {
            this.range = range;
        }

        public RangeBuilder appliesTo(OutputTarget target) {
            this.target = Objects.requireNonNull(target, "target");
            return this;
        }

        public RuleRef warn() {
            return build(ctx -> RuleResult.warn());
        }

        public RuleRef warn(String reason) {
            return build(ctx -> RuleResult.warn(reason));
        }

        public RuleRef block() {
            return build(ctx -> RuleResult.block());
        }

        public RuleRef block(String reason) {
            return build(ctx -> RuleResult.block(reason));
        }

        public RuleRef clamp() {
            return build(ctx -> RuleResult.clamp(range.clamp(ctx.request().value())));
        }

        public RuleRef clamp(String reason) {
            return build(ctx -> RuleResult.clamp(range.clamp(ctx.request().value()), reason));
        }

        private RuleRef build(Function<RuleContext, RuleResult> result) {
            return new RuleRef(
                    ctx -> ctx.request().value() < range.minimum() || ctx.request().value() > range.maximum(),
                    target,
                    result,
                    "");
        }
    }
}
