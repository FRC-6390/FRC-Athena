package ca.frc6390.athena.commands;

import java.util.Objects;
import java.util.Set;

/**
 * Immutable command descriptor.
 *
 * @param name command name
 * @param onInitialize action run when command starts
 * @param onExecute action run each command cycle
 * @param isFinished finish condition
 * @param onEnd action run when command ends
 * @param requirements named subsystem or resource requirements
 */
public record CommandSpec(
        String name,
        AthenaAction onInitialize,
        AthenaAction onExecute,
        AthenaCondition isFinished,
        AthenaAction onEnd,
        Set<String> requirements) {
    private static final AthenaAction NO_OP_ACTION = () -> {
    };
    private static final AthenaCondition NEVER_FINISHED = () -> false;

    public CommandSpec {
        name = name == null || name.isBlank() ? "command" : name;
        onInitialize = onInitialize == null ? NO_OP_ACTION : onInitialize;
        onExecute = onExecute == null ? NO_OP_ACTION : onExecute;
        isFinished = isFinished == null ? NEVER_FINISHED : isFinished;
        onEnd = onEnd == null ? NO_OP_ACTION : onEnd;
        requirements = requirements == null ? Set.of() : Set.copyOf(requirements);
        Objects.requireNonNull(name, "name");
    }

    /**
     * Creates a command builder.
     *
     * @param name command name
     * @return command builder
     */
    public static CommandSpecBuilder create(String name) {
        return new CommandSpecBuilder(name);
    }
}
