package ca.frc6390.athena.api.superstructure.definition;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

public record SuperstructureDefinition<SP>(
    String name,
    Class<?> stateType,
    String initialStateName,
    Optional<Object> initialState,
    double stateMachineDelaySeconds,
    List<SuperstructureChildDefinition<SP>> children,
    List<SuperstructureInputDefinition> inputs,
    List<SuperstructureConstraintDefinition<SP>> constraints,
    List<SuperstructureHookDefinition<SP>> hooks,
    List<SuperstructureTransitionHookDefinition<SP>> transitionHooks
) {
    public SuperstructureDefinition {
        name = name == null ? "" : name;
        stateType = Objects.requireNonNull(stateType, "stateType");
        initialStateName = Objects.requireNonNull(initialStateName, "initialStateName");
        initialState = Objects.requireNonNull(initialState, "initialState");
        children = List.copyOf(Objects.requireNonNull(children, "children"));
        inputs = List.copyOf(Objects.requireNonNull(inputs, "inputs"));
        constraints = List.copyOf(Objects.requireNonNull(constraints, "constraints"));
        hooks = List.copyOf(Objects.requireNonNull(hooks, "hooks"));
        transitionHooks = List.copyOf(Objects.requireNonNull(transitionHooks, "transitionHooks"));
    }

    public SuperstructureDefinition(
            String name,
            Class<?> stateType,
            String initialStateName,
            double stateMachineDelaySeconds,
            List<SuperstructureChildDefinition<SP>> children,
            List<SuperstructureInputDefinition> inputs,
            List<SuperstructureConstraintDefinition<SP>> constraints,
            List<SuperstructureHookDefinition<SP>> hooks,
            List<SuperstructureTransitionHookDefinition<SP>> transitionHooks) {
        this(
            name,
            stateType,
            initialStateName,
            Optional.empty(),
            stateMachineDelaySeconds,
            children,
            inputs,
            constraints,
            hooks,
            transitionHooks);
    }
}
