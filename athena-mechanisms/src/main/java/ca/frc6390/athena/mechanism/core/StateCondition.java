package ca.frc6390.athena.mechanism.core;

/**
 * Predicate evaluated by the mechanism runtime while a state is active.
 */
@FunctionalInterface
public interface StateCondition {
    /**
     * Returns true when the condition is satisfied.
     *
     * @param ctx current mechanism context
     * @return true when satisfied
     */
    boolean test(MechanismContext ctx);
}
