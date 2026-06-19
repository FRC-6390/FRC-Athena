package ca.frc6390.athena.api.superstructure.behavior.constraint;

@FunctionalInterface
public interface ConstraintsConfigurer<S, SP> {
    void apply(SuperstructureConstraints<S, SP> constraints);
}
