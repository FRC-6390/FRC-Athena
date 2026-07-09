package ca.frc6390.athena.commands;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Runtime owner for command state lifecycle and requirement arbitration.
 */
public final class CommandGraph {
    private final Map<String, ActiveCommand> activeCommands = new LinkedHashMap<>();
    private final Map<String, String> requirementOwners = new LinkedHashMap<>();

    /**
     * Schedules a command state.
     *
     * @param state command state
     * @return this graph
     */
    public CommandGraph schedule(CommandState state) {
        CommandState safeState = Objects.requireNonNull(state, "state");
        cancelConflicts(safeState.requirements());
        ActiveCommand existing = activeCommands.remove(safeState.name());
        if (existing != null) {
            existing.end(true);
            releaseRequirements(existing.state());
        }
        ActiveCommand active = new ActiveCommand(safeState);
        activeCommands.put(safeState.name(), active);
        for (String requirement : safeState.requirements()) {
            requirementOwners.put(requirement, safeState.name());
        }
        return this;
    }

    /**
     * Executes one scheduler cycle.
     *
     * @return command names that finished this cycle
     */
    public Collection<String> periodic() {
        Set<String> finished = new LinkedHashSet<>();
        for (ActiveCommand active : ListSnapshot.copyOf(activeCommands.values())) {
            if (active.execute()) {
                finished.add(active.state().name());
                activeCommands.remove(active.state().name());
                releaseRequirements(active.state());
            }
        }
        return finished;
    }

    /**
     * Cancels a command by name.
     *
     * @param name command name
     * @return true when a command was cancelled
     */
    public boolean cancel(String name) {
        ActiveCommand active = activeCommands.remove(normalize(name));
        if (active == null) {
            return false;
        }
        active.end(true);
        releaseRequirements(active.state());
        return true;
    }

    /**
     * Cancels all active commands.
     *
     * @return this graph
     */
    public CommandGraph cancelAll() {
        for (ActiveCommand active : activeCommands.values()) {
            active.end(true);
        }
        activeCommands.clear();
        requirementOwners.clear();
        return this;
    }

    /**
     * Returns active command names.
     *
     * @return active command names
     */
    public Collection<String> activeCommandNames() {
        return activeCommands.keySet();
    }

    private void cancelConflicts(Collection<String> requirements) {
        Set<String> conflicts = new LinkedHashSet<>();
        for (String requirement : requirements) {
            String owner = requirementOwners.get(requirement);
            if (owner != null) {
                conflicts.add(owner);
            }
        }
        conflicts.forEach(this::cancel);
    }

    private void releaseRequirements(CommandState state) {
        for (String requirement : state.requirements()) {
            requirementOwners.remove(requirement, state.name());
        }
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "command" : name.trim();
    }

    private record ActiveCommand(CommandState state) {
        private ActiveCommand {
            Objects.requireNonNull(state, "state");
            state.onInitialize().run();
        }

        private boolean execute() {
            state.onExecute().run();
            if (state.isFinished().getAsBoolean()) {
                end(false);
                return true;
            }
            return false;
        }

        private void end(boolean interrupted) {
            state.onEnd().run();
        }
    }

    private static final class ListSnapshot {
        private static <T> Collection<T> copyOf(Collection<T> values) {
            return java.util.List.copyOf(values);
        }
    }
}
