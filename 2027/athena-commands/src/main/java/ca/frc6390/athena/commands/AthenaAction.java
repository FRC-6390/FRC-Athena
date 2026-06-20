package ca.frc6390.athena.commands;

/**
 * Small executable action used by Athena command descriptors.
 *
 * <p>This interface intentionally avoids WPILib dependencies. A later adapter
 * can map descriptors to WPILib commands without forcing that dependency into
 * the base command artifact.</p>
 */
@FunctionalInterface
public interface AthenaAction {
    /**
     * Executes one action step.
     */
    void run();
}
