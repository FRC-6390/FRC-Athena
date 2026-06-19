package ca.frc6390.athena.api.mechanism.behavior;

@FunctionalInterface
public interface BehaviorConfigurer {
    MechanismBehavior apply(MechanismBehavior behavior);
}
