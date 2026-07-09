package ca.frc6390.athena.commands;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Runtime owner for command Action lifecycle and requirement arbitration.
 */
public final class CommandGraph {
    private final Map<String, ActiveCommand> activeCommands = new LinkedHashMap<>();
    private final Map<String, String> requirementOwners = new LinkedHashMap<>();

    /**
     * Schedules a command Action.
     *
     * @param Action command Action
     * @return this graph
     */
    public CommandGraph schedule(CommandAction Action) {
        CommandAction safeState = Objects.requireNonNull(Action, "Action");
        cancelConflicts(safeState.requirements());
        ActiveCommand existing = activeCommands.remove(safeState.name());
        if (existing != null) {
            existing.end(true);
            releaseRequirements(existing.Action());
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
                finished.add(active.Action().name());
                activeCommands.remove(active.Action().name());
                releaseRequirements(active.Action());
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
        releaseRequirements(active.Action());
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

    private void releaseRequirements(CommandAction Action) {
        for (String requirement : Action.requirements()) {
            requirementOwners.remove(requirement, Action.name());
        }
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "command" : name.trim();
    }

    private record ActiveCommand(CommandAction Action) {
        private ActiveCommand {
            Objects.requireNonNull(Action, "Action");
            Action.onInitialize().run();
        }

        private boolean execute() {
            Action.onExecute().run();
            if (Action.isFinished().getAsBoolean()) {
                end(false);
                return true;
            }
            return false;
        }

        private void end(boolean interrupted) {
            Action.onEnd().run();
        }
    }

    private static final class ListSnapshot {
        private static <T> Collection<T> copyOf(Collection<T> values) {
            return java.util.List.copyOf(values);
        }
    }
}
