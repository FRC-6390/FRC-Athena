package ca.frc6390.athena.api.mechanism.behavior.control;

@FunctionalInterface
public interface ControlConfigurer {
    MechanismControl apply(MechanismControl control);
}
