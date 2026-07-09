package ca.frc6390.athena.mechanism.core;

/**
 * Predicate evaluated against mechanism runtime context.
 */
@FunctionalInterface
public interface StateCondition {
    boolean test(MechanismContext context);
}
