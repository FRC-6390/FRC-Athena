package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandState;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathState;

/**
 * External provider for path-backed autonomous states and mechanism path runtimes.
 */
public interface PathProvider {
    /**
     * Creates a path state for the provider.
     *
     * @param pathName provider-specific path name
     * @return path state
     */
    PathState path(String pathName);

    /**
     * Loads a path as a command state.
     *
     * @param pathName provider-specific path name
     * @return command state for the path
     */
    CommandState load(String pathName);

    /**
     * Creates the mechanism path runtime that executes provider path states.
     *
     * @return path runtime
     */
    PathRuntime runtime();
}
