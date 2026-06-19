package ca.frc6390.athena.api.mechanism;

import java.util.Optional;

public final class StatefulFlowMechanismConfig<S> extends FlowMechanismConfig
    implements TypedStatefulMechanismConfig<S> {
    private final S initialState;

    public StatefulFlowMechanismConfig(String name, S initialState) {
        super(name);
        this.initialState = initialState;
        state(Optional.of(initialState.getClass()), Optional.of(ca.frc6390.athena.mechanisms.statespec.StateNames.name(initialState)));
        initialState(Optional.of(initialState));
    }

    @Override
    public StatefulFlowMechanismConfig<S> disabled(boolean disabled) {
        super.disabled(disabled);
        return this;
    }

    @Override
    public S initialState() {
        return initialState;
    }

    @Override
    public StatefulFlowMechanismConfig<S> stateMachineDelaySeconds(double delaySeconds) {
        super.stateMachineDelaySeconds(delaySeconds);
        return this;
    }
}
