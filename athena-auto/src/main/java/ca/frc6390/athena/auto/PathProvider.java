package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;

/**
 * External provider for path-backed autonomous Actions and mechanism path runtimes.
 */
public interface PathProvider {
    /**
     * Creates a path Action for the provider.
     *
     * @param pathName provider-specific path name
     * @return path Action
     */
    PathAction path(String pathName);

    /**
     * Loads a path as a command Action.
     *
     * @param pathName provider-specific path name
     * @return command Action for the path
     */
    CommandAction load(String pathName);

    /**
     * Creates the mechanism path runtime that executes provider path Actions.
     *
     * @return path runtime
     */
    PathRuntime runtime();
}
