package ca.frc6390.athena.api.mechanism.input;

@FunctionalInterface
public interface InputsConfigurer {
    MechanismInputs apply(MechanismInputs inputs);
}
