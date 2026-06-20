package ca.frc6390.athena.commands;

import java.util.List;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Factory methods for composed command specs.
 */
public final class CommandGroups {
    private CommandGroups() {
    }

    /**
     * Creates a command that runs children one after another.
     *
     * @param name group name
     * @param commands child command specs
     * @return composed command spec
     */
    public static CommandSpec sequence(String name, CommandSpec... commands) {
        return sequence(name, List.of(commands));
    }

    /**
     * Creates a command that runs children one after another.
     *
     * @param name group name
     * @param commands child command specs
     * @return composed command spec
     */
    public static CommandSpec sequence(String name, List<CommandSpec> commands) {
        List<CommandRunner> runners = copyCommands(commands).stream()
                .map(CommandRunner::new)
                .toList();
        final class SequenceState {
            private int index;

            void execute() {
                if (index >= runners.size()) {
                    return;
                }
                if (runners.get(index).step()) {
                    index++;
                }
            }

            boolean finished() {
                return index >= runners.size();
            }
        }
        SequenceState state = new SequenceState();
        return CommandSpec.create(name)
                .onExecute(state::execute)
                .until(state::finished)
                .requires(requirementsOf(commands))
                .toSpec();
    }

    /**
     * Creates a command that runs all children until every child finishes.
     *
     * @param name group name
     * @param commands child command specs
     * @return composed command spec
     */
    public static CommandSpec parallel(String name, CommandSpec... commands) {
        return parallel(name, List.of(commands));
    }

    /**
     * Creates a command that runs all children until every child finishes.
     *
     * @param name group name
     * @param commands child command specs
     * @return composed command spec
     */
    public static CommandSpec parallel(String name, List<CommandSpec> commands) {
        List<CommandRunner> runners = copyCommands(commands).stream()
                .map(CommandRunner::new)
                .toList();
        boolean[] finished = new boolean[runners.size()];
        return CommandSpec.create(name)
                .onExecute(() -> {
                    for (int index = 0; index < runners.size(); index++) {
                        if (!finished[index]) {
                            finished[index] = runners.get(index).step();
                        }
                    }
                })
                .until(() -> {
                    for (boolean childFinished : finished) {
                        if (!childFinished) {
                            return false;
                        }
                    }
                    return true;
                })
                .requires(requirementsOf(commands))
                .toSpec();
    }

    private static Set<String> requirementsOf(List<CommandSpec> commands) {
        Set<String> requirements = new LinkedHashSet<>();
        for (CommandSpec command : commands) {
            requirements.addAll(command.requirements());
        }
        return requirements;
    }

    private static List<CommandSpec> copyCommands(List<CommandSpec> commands) {
        Objects.requireNonNull(commands, "commands");
        if (commands.isEmpty()) {
            throw new IllegalArgumentException("Command group must include at least one child command.");
        }
        commands.forEach(command -> Objects.requireNonNull(command, "command"));
        return List.copyOf(commands);
    }
}
