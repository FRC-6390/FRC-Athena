package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandSpec;

/**
 * Provider capable of loading autonomous commands from an engine-specific path.
 */
@FunctionalInterface
public interface AutoSource {
    /**
     * Loads an autonomous command descriptor.
     *
     * @param routinePath engine-specific routine path
     * @return command descriptor
     */
    CommandSpec load(String routinePath);
}
