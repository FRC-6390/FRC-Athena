package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Declarative rule for validating or modifying mechanism output requests.
 */
public record RuleRef(
        Predicate<RuleContext> condition,
        OutputTarget target,
        Function<RuleContext, RuleResult> result,
        String name) {
    public RuleRef {
        condition = condition == null ? ctx -> true : condition;
        target = target == null ? OutputTarget.any() : target;
        result = result == null ? ctx -> RuleResult.allow() : result;
        name = name == null ? "" : name;
    }

    public RuleRef appliesTo(OutputTarget target) {
        return new RuleRef(condition, Objects.requireNonNull(target, "target"), result, name);
    }

    public RuleRef named(String name) {
        return new RuleRef(condition, target, result, name);
    }

    public RuleResult evaluate(RuleContext context) {
        Objects.requireNonNull(context, "context");
        if (!target.matches(context.request()) || !condition.test(context)) {
            return RuleResult.allow();
        }
        return result.apply(context);
    }
}
