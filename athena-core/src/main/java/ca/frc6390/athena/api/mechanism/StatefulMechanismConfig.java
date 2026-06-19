package ca.frc6390.athena.api.mechanism;

import ca.frc6390.athena.mechanisms.statespec.StateId;

public interface StatefulMechanismConfig extends MechanismConfig {
    StateId initialState();

    default double stateMachineDelaySeconds() {
        return 0.0;
    }

    default Class<?> stateType() {
        StateId initialState = initialState();
        return initialState != null ? initialState.getClass() : null;
    }

    @Override
    default ca.frc6390.athena.api.mechanism.definition.MechanismDefinition definition() {
        return MechanismDefinitions.structured(this);
    }
}
