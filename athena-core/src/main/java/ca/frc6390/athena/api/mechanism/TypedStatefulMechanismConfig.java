package ca.frc6390.athena.api.mechanism;

public interface TypedStatefulMechanismConfig<S> extends MechanismConfig {
    S initialState();

    default double stateMachineDelaySeconds() {
        return 0.0;
    }

    default Class<?> stateType() {
        Object initialState = initialState();
        return initialState != null ? initialState.getClass() : null;
    }

    @Override
    default ca.frc6390.athena.api.mechanism.definition.MechanismDefinition definition() {
        return MechanismDefinitions.structured(this);
    }
}
