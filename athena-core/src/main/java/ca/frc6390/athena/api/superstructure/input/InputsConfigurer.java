package ca.frc6390.athena.api.superstructure.input;

@FunctionalInterface
public interface InputsConfigurer {
    void apply(SuperstructureInputs inputs);
}
