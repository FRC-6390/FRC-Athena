package ca.frc6390.athena.api.mechanism;

public final class Mechanisms {
    private Mechanisms() {
    }

    public static FlowMechanismConfig create(String name) {
        return new FlowMechanismConfig(name);
    }

    public static <S> StatefulFlowMechanismConfig<S> stateful(String name, S initialState) {
        return new StatefulFlowMechanismConfig<>(name, initialState);
    }
}
