package ca.frc6390.athena.api.mechanism.motor;

@FunctionalInterface
public interface MotorsConfigurer {
    MechanismMotors apply(MechanismMotors motors);
}
