package ca.frc6390.athena.mechanism.core;

import java.util.Objects;

/**
 * Final output after Athena resolves a state.
 */
public record ResolvedOutput(OutputRequest request, Output output) {
    public ResolvedOutput {
        Objects.requireNonNull(request, "request");
        Objects.requireNonNull(output, "output");
    }
}
