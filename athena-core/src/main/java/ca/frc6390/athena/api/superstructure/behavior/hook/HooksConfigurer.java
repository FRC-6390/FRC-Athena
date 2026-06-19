package ca.frc6390.athena.api.superstructure.behavior.hook;

@FunctionalInterface
public interface HooksConfigurer<S, SP> {
    void apply(SuperstructureHooks<S, SP> hooks);
}
