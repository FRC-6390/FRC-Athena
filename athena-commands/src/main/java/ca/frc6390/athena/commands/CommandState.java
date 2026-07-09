package ca.frc6390.athena.commands;

import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Command behavior state that can be adapted by a robot runtime or external command framework.
 *
 * @param name command state name
 * @param onInitialize action run when the command state starts
 * @param onExecute action run each command state cycle
 * @param isFinished finish condition
 * @param onEnd action run when the command state ends
 * @param requirements named subsystem or resource requirements
 */
public record CommandState(
        String name,
        Runnable onInitialize,
        Runnable onExecute,
        BooleanSupplier isFinished,
        Runnable onEnd,
        Set<String> requirements) {
    private static final Runnable NO_OP = () -> {
    };
    private static final BooleanSupplier NEVER_FINISHED = () -> false;

    public CommandState {
        name = name == null || name.isBlank() ? "command" : name.trim();
        onInitialize = onInitialize == null ? NO_OP : onInitialize;
        onExecute = onExecute == null ? NO_OP : onExecute;
        isFinished = isFinished == null ? NEVER_FINISHED : isFinished;
        onEnd = onEnd == null ? NO_OP : onEnd;
        requirements = requirements == null ? Set.of() : Set.copyOf(requirements);
        Objects.requireNonNull(name, "name");
    }

    /**
     * Creates a command state builder.
     *
     * @param name command state name
     * @return command state builder
     */
    public static CommandStateBuilder create(String name) {
        return new CommandStateBuilder(name);
    }
}
