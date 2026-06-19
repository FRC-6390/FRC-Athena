package ca.frc6390.athena.api.superstructure;

public final class Superstructures {
    private Superstructures() {
    }

    public static <S, SP> FlowSuperstructureConfig<S, SP> stateful(S initialState) {
        return new FlowSuperstructureConfig<>("", initialState);
    }

    public static <S, SP> FlowSuperstructureConfig<S, SP> stateful(String name, S initialState) {
        return new FlowSuperstructureConfig<>(name, initialState);
    }
}
