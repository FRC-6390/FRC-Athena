package ca.frc6390.athena.api.mechanism.definition;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;

public record LoopActivation(
    LoopMode mode,
    List<String> states
) {
    public LoopActivation {
        mode = Objects.requireNonNull(mode, "mode");
        states = List.copyOf(Objects.requireNonNull(states, "states"));
    }

    public static LoopActivation enabled() {
        return new LoopActivation(LoopMode.ENABLED, List.of());
    }
}
