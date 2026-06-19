package ca.frc6390.athena.api.superstructure.behavior;

@FunctionalInterface
public interface BehaviorConfigurer<S, SP> {
    void apply(SuperstructureBehavior<S, SP> behavior);
}
