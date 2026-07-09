package ca.frc6390.athena.mechanism.core;

/**
 * Predicate evaluated against mechanism runtime context.
 */
@FunctionalInterface
public interface ActionCondition {
    boolean test(MechanismContext context);
}
