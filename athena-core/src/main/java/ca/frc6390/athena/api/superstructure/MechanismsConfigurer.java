package ca.frc6390.athena.api.superstructure;

@FunctionalInterface
public interface MechanismsConfigurer<SP> {
    SuperstructureMechanisms<SP> apply(SuperstructureMechanisms<SP> mechanisms);
}
