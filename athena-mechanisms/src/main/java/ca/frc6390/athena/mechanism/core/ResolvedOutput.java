package ca.frc6390.athena.mechanism.core;

import java.util.List;
import java.util.Objects;

/**
 * Final output after Athena evaluates matching rules.
 */
public record ResolvedOutput(OutputRequest request, Output output, List<RuleResult> results) {
    public ResolvedOutput {
        Objects.requireNonNull(request, "request");
        Objects.requireNonNull(output, "output");
        results = results == null ? List.of() : List.copyOf(results);
    }

    public boolean blocked() {
        return results.stream().anyMatch(result -> result.decision() == RuleResult.Decision.BLOCK);
    }

    public boolean clamped() {
        return !blocked() && results.stream().anyMatch(result -> result.decision() == RuleResult.Decision.CLAMP);
    }

    public boolean warned() {
        return results.stream().anyMatch(result -> result.decision() == RuleResult.Decision.WARN);
    }

    public List<String> messages() {
        return results.stream()
                .filter(result -> result.decision() != RuleResult.Decision.ALLOW)
                .map(RuleResult::message)
                .toList();
    }
}
