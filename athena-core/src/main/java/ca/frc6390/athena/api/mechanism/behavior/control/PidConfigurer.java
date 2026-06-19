package ca.frc6390.athena.api.mechanism.behavior.control;

@FunctionalInterface
public interface PidConfigurer {
    MechanismPid apply(MechanismPid pid);
}
