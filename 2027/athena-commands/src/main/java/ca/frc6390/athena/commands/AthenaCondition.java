package ca.frc6390.athena.commands;

/**
 * Boolean condition used by command descriptors.
 */
@FunctionalInterface
public interface AthenaCondition {
    /**
     * Evaluates the condition.
     *
     * @return true when the condition is satisfied
     */
    boolean get();
}
