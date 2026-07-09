package ca.frc6390.athena.commands;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Command behavior Action that can be adapted by a robot runtime or external command framework.
 *
 * @param name command Action name
 * @param onInitialize action run when the command Action starts
 * @param onExecute action run each command Action cycle
 * @param isFinished finish condition
 * @param onEnd action run when the command Action ends
 * @param requirements named subsystem or resource requirements
 */
public record CommandAction(
        String name,
        Runnable onInitialize,
        Runnable onExecute,
        BooleanSupplier isFinished,
        Runnable onEnd,
        Set<String> requirements) {
    private static final Runnable NO_OP = () -> {
    };
    private static final BooleanSupplier NEVER_FINISHED = () -> false;

    public CommandAction {
        name = name == null || name.isBlank() ? "command" : name.trim();
        onInitialize = onInitialize == null ? NO_OP : onInitialize;
        onExecute = onExecute == null ? NO_OP : onExecute;
        isFinished = isFinished == null ? NEVER_FINISHED : isFinished;
        onEnd = onEnd == null ? NO_OP : onEnd;
        requirements = requirements == null ? Set.of() : Set.copyOf(requirements);
        Objects.requireNonNull(name, "name");
    }

    /**
     * Creates a command Action builder.
     *
     * @param name command Action name
     * @return command Action builder
     */
    public static CommandActionBuilder create(String name) {
        return new CommandActionBuilder(name);
    }
}
