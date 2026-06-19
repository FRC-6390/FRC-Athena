package ca.frc6390.athena.api.superstructure.definition;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.api.superstructure.behavior.hook.SuperstructureHookCallback;
import ca.frc6390.athena.core.RobotCoreHooks;

public record SuperstructureHookDefinition<SP>(
    SuperstructureHookPhase phase,
    Optional<RobotCoreHooks.Phase> robotPhase,
    List<String> states,
    SuperstructureHookCallback<SP> callback
) {
    public SuperstructureHookDefinition {
        phase = Objects.requireNonNull(phase, "phase");
        robotPhase = Objects.requireNonNull(robotPhase, "robotPhase");
        states = List.copyOf(Objects.requireNonNull(states, "states"));
        callback = Objects.requireNonNull(callback, "callback");
    }
}
