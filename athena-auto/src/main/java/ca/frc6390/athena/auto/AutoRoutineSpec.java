package ca.frc6390.athena.auto;

import java.util.Objects;

import ca.frc6390.athena.commands.CommandSpec;

/**
 * Immutable autonomous routine declaration.
 *
 * @param id stable chooser id
 * @param displayName dashboard/display name
 * @param command command descriptor
 */
public record AutoRoutineSpec(String id, String displayName, CommandSpec command) {
    public AutoRoutineSpec {
        id = id == null || id.isBlank() ? "auto" : id;
        displayName = displayName == null || displayName.isBlank() ? id : displayName;
        Objects.requireNonNull(command, "command");
    }
}
