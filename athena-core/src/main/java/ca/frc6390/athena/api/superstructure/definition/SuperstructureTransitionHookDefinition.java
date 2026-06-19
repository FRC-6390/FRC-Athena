package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;

import ca.frc6390.athena.api.superstructure.behavior.hook.SuperstructureTransitionHookCallback;

public record SuperstructureTransitionHookDefinition<SP>(
    String fromState,
    String toState,
    SuperstructureTransitionHookCallback<SP> callback
) {
    public SuperstructureTransitionHookDefinition {
        fromState = Objects.requireNonNull(fromState, "fromState");
        toState = Objects.requireNonNull(toState, "toState");
        callback = Objects.requireNonNull(callback, "callback");
    }
}
