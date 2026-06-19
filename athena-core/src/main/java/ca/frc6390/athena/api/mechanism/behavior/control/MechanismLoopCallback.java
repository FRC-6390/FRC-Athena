package ca.frc6390.athena.api.mechanism.behavior.control;

@FunctionalInterface
public interface MechanismLoopCallback {
    double calculate(MechanismLoopContext context);
}
