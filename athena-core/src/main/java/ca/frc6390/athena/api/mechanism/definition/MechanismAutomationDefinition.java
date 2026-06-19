package ca.frc6390.athena.api.mechanism.definition;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.mechanism.behavior.automation.MechanismStateHookCallback;

public record MechanismAutomationDefinition(
    MechanismAutomationPhase phase,
    List<String> states,
    MechanismStateHookCallback callback
) {
    public MechanismAutomationDefinition {
        phase = Objects.requireNonNull(phase, "phase");
        states = List.copyOf(Objects.requireNonNull(states, "states"));
        callback = Objects.requireNonNull(callback, "callback");
    }
}
