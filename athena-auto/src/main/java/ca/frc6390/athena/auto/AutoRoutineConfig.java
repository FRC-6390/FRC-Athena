package ca.frc6390.athena.auto;

import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.commands.CommandSpec;

/**
 * Student-facing autonomous routine declaration.
 */
public final class AutoRoutineConfig {
    private final String id;
    private String displayName;
    private Supplier<CommandSpec> commandFactory;

    AutoRoutineConfig(String id) {
        this.id = id;
        this.displayName = id;
    }

    /**
     * Sets display name.
     *
     * @param displayName display name
     * @return this config
     */
    public AutoRoutineConfig displayName(String displayName) {
        this.displayName = displayName;
        return this;
    }

    /**
     * Uses an explicit command descriptor.
     *
     * @param command command descriptor
     * @return this config
     */
    public AutoRoutineConfig command(CommandSpec command) {
        commandFactory = () -> Objects.requireNonNull(command, "command");
        return this;
    }

    /**
     * Loads a command from a registered auto source.
     *
     * @param sourceKey source key
     * @param routinePath engine-specific routine path
     * @return this config
     */
    public AutoRoutineConfig fromSource(String sourceKey, String routinePath) {
        commandFactory = () -> AutoRegistry.get().require(sourceKey).load(routinePath);
        return this;
    }

    AutoRoutineSpec toSpec() {
        if (commandFactory == null) {
            throw new IllegalStateException("Auto routine '" + id + "' needs a command or source.");
        }
        return new AutoRoutineSpec(id, displayName, commandFactory.get());
    }
}
