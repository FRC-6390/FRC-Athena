package ca.frc6390.athena.api.superstructure;

import ca.frc6390.athena.api.superstructure.behavior.SuperstructureBehavior;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.api.superstructure.input.SuperstructureInputs;

public interface SuperstructureConfig<S, SP> {
    S initialState();

    default double stateMachineDelaySeconds() {
        return 0.0;
    }

    default String name() {
        return "";
    }

    default void mechanisms(SuperstructureMechanisms<SP> mechanisms) {
    }

    default void inputs(SuperstructureInputs inputs) {
    }

    default void behavior(SuperstructureBehavior<S, SP> behavior) {
    }

    default SuperstructureDefinition<SP> definition() {
        return SuperstructureDefinitions.structured(this);
    }
}
