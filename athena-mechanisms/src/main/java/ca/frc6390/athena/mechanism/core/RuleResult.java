package ca.frc6390.athena.mechanism.core;

import java.util.OptionalDouble;

/**
 * Result returned by a rule evaluation.
 */
public record RuleResult(Decision decision, OptionalDouble value, String reason) {
    public RuleResult {
        decision = decision == null ? Decision.ALLOW : decision;
        value = value == null ? OptionalDouble.empty() : value;
        reason = reason == null ? "" : reason;
    }

    public static RuleResult allow() {
        return new RuleResult(Decision.ALLOW, OptionalDouble.empty(), "");
    }

    public static RuleResult warn() {
        return warn("");
    }

    public static RuleResult warn(String reason) {
        return new RuleResult(Decision.WARN, OptionalDouble.empty(), reason);
    }

    public static RuleResult block() {
        return block("");
    }

    public static RuleResult block(String reason) {
        return new RuleResult(Decision.BLOCK, OptionalDouble.empty(), reason);
    }

    public static RuleResult clamp(double value) {
        return clamp(value, "");
    }

    public static RuleResult clamp(double value, String reason) {
        return new RuleResult(Decision.CLAMP, OptionalDouble.of(value), reason);
    }

    public String message() {
        String base = switch (decision) {
            case ALLOW -> "mechanism request was allowed";
            case WARN -> "mechanism request produced warning";
            case BLOCK -> "mechanism request was blocked";
            case CLAMP -> "mechanism request was clamped";
        };
        return reason.isBlank() ? base : base + ": " + reason;
    }

    public enum Decision {
        ALLOW,
        WARN,
        BLOCK,
        CLAMP
    }
}
