package ca.frc6390.athena.commands;

import java.util.ArrayDeque;
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
    private final ArrayDeque<Runnable> deferredMutations = new ArrayDeque<>();
    private boolean mutating;

    /**
     * Schedules a command Action.
     *
     * @param Action command Action
     * @return this graph
     */
    public CommandGraph schedule(CommandAction Action) {
        CommandAction safeState = Objects.requireNonNull(Action, "Action");
        mutate(() -> scheduleNow(safeState));
        return this;
    }

    private void scheduleNow(CommandAction safeState) {
        cancelConflictsNow(safeState.requirements());
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
    }

    /**
     * Executes one scheduler cycle.
     *
     * @return command names that finished this cycle
     */
    public Collection<String> periodic() {
        Set<String> finished = new LinkedHashSet<>();
        mutate(() -> {
            for (ActiveCommand active : ListSnapshot.copyOf(activeCommands.values())) {
                if (active.execute()) {
                    finished.add(active.Action().name());
                    if (activeCommands.remove(active.Action().name(), active)) {
                        active.end(false);
                        releaseRequirements(active.Action());
                    }
                }
            }
        });
        return finished;
    }

    /**
     * Cancels a command by name.
     *
     * @param name command name
     * @return true when a command was cancelled
     */
    public boolean cancel(String name) {
        String normalized = normalize(name);
        boolean wasActive = activeCommands.containsKey(normalized);
        mutate(() -> cancelNow(normalized));
        return wasActive;
    }

    private boolean cancelNow(String name) {
        ActiveCommand active = activeCommands.remove(name);
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
        mutate(this::cancelAllNow);
        return this;
    }

    private void cancelAllNow() {
        Collection<ActiveCommand> cancelled = ListSnapshot.copyOf(activeCommands.values());
        activeCommands.clear();
        requirementOwners.clear();
        cancelled.forEach(active -> active.end(true));
    }

    /**
     * Returns active command names.
     *
     * @return active command names
     */
    public Collection<String> activeCommandNames() {
        return java.util.List.copyOf(activeCommands.keySet());
    }

    private void cancelConflictsNow(Collection<String> requirements) {
        Set<String> conflicts = new LinkedHashSet<>();
        for (String requirement : requirements) {
            String owner = requirementOwners.get(requirement);
            if (owner != null) {
                conflicts.add(owner);
            }
        }
        conflicts.forEach(this::cancelNow);
    }

    /**
     * Serializes graph mutations initiated by lifecycle callbacks. A callback-triggered mutation
     * takes effect immediately after the lifecycle transition that triggered it, preserving the
     * single-owner requirement invariant without modifying a collection during iteration.
     */
    private void mutate(Runnable mutation) {
        if (mutating) {
            deferredMutations.addLast(mutation);
            return;
        }
        mutating = true;
        try {
            mutation.run();
            while (!deferredMutations.isEmpty()) {
                deferredMutations.removeFirst().run();
            }
        } catch (RuntimeException | Error failure) {
            deferredMutations.clear();
            throw failure;
        } finally {
            mutating = false;
        }
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
            return Action.isFinished().getAsBoolean();
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
