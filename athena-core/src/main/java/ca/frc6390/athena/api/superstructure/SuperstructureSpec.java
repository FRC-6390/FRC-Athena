package ca.frc6390.athena.api.superstructure;

import java.util.List;
import java.util.Optional;

import ca.frc6390.athena.api.superstructure.behavior.SuperstructureBehavior;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.api.superstructure.input.SuperstructureInputs;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class SuperstructureSpec<S, SP> {
    private String name = "";
    private S initialState;
    private double stateMachineDelaySeconds;
    private final SuperstructureMechanisms<SP> mechanisms = SuperstructureMechanisms.create();
    private final SuperstructureInputs inputs = SuperstructureInputs.create();
    private final SuperstructureBehavior<S, SP> behavior = SuperstructureBehavior.create();

    public SuperstructureSpec<S, SP> name(String name) {
        this.name = name != null ? name : "";
        return this;
    }

    public SuperstructureSpec<S, SP> initialState(S initialState) {
        this.initialState = initialState;
        return this;
    }

    public SuperstructureSpec<S, SP> stateMachineDelaySeconds(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
        return this;
    }

    public SuperstructureMechanisms<SP> mechanisms() {
        return mechanisms;
    }

    public SuperstructureInputs inputs() {
        return inputs;
    }

    public SuperstructureBehavior<S, SP> behavior() {
        return behavior;
    }

    public SuperstructureDefinition<SP> definition() {
        if (initialState == null) {
            throw new IllegalStateException("superstructure initial state is required");
        }
        return new SuperstructureDefinition<>(
            name,
            initialState.getClass(),
            StateNames.name(initialState),
            Optional.of(initialState),
            stateMachineDelaySeconds,
            List.copyOf(mechanisms.definitions()),
            inputs.definitions(),
            behavior.constraintDefinitions(),
            behavior.hookDefinitions(),
            behavior.transitionDefinitions());
    }
}
