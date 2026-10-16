package ca.frc6390.athena.mechanism.core;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * External imperative behaviour as an Athena Action: start, per-cycle execute, a finish condition, and an
 * end callback. It runs inside the mechanism scheduler like any other Action, so it composes with
 * sequence, parallel, race, timeout, and the rest, and its requirements are arbitrated by the same
 * leases.
 *
 * <p>Lifecycle order matches WPILib commands: {@code onStart} runs on the first cycle the Action is
 * evaluated, {@code execute} runs every cycle, and {@code until} is checked after execute. When it
 * becomes true the Action completes and {@code onEnd} receives {@code false}. When the Action is
 * cancelled, replaced by a newer conflicting request, times out, loses a race, or the robot is disabled,
 * {@code onEnd} receives {@code true}. A disabled robot does not run {@code execute} or {@code until}; the
 * Action restarts with {@code onStart} when enabled again if its request is still held.
 *
 * <p>Requirements are Athena declarations (mechanisms, devices, controls, which select the owning
 * mechanism root and its resources) or {@link NamedResource}s. A newer request that overlaps a running
 * lifecycle Action's resources ends that Action, so two lifecycle Actions sharing a requirement never run
 * together.
 *
 * @param name diagnostic name
 * @param onStart runs when the Action first starts (never null after construction)
 * @param onExecute runs each cycle while running (never null after construction)
 * @param isFinished finish condition, or null when the Action never finishes on its own
 * @param onEnd receives {@code true} when interrupted and {@code false} when finished
 * @param requirements Athena declarations and named resources this Action requires
 */
public record LifecycleAction(
        String name,
        Runnable onStart,
        Runnable onExecute,
        BooleanSupplier isFinished,
        Consumer<Boolean> onEnd,
        Set<Object> requirements) implements Action {
    private static final Runnable NO_OP = () -> {
    };

    public LifecycleAction {
        name = name == null || name.isBlank() ? "lifecycle" : name.trim();
        onStart = onStart == null ? NO_OP : onStart;
        onExecute = onExecute == null ? NO_OP : onExecute;
        onEnd = onEnd == null ? interrupted -> { } : onEnd;
        requirements = requirements == null
                ? Set.of()
                : Collections.unmodifiableSet(new LinkedHashSet<>(requirements));
        for (Object requirement : requirements) Objects.requireNonNull(requirement, "requirement");
    }

    /**
     * Starts a lifecycle Action builder. The name is diagnostic only.
     *
     * @param name diagnostic name
     * @return builder
     */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /** Returns whether this Action can finish on its own, which makes it bounded. */
    public boolean finishes() {
        return isFinished != null;
    }

    void initialize() {
        onStart.run();
    }

    void execute() {
        onExecute.run();
    }

    boolean finished() {
        return isFinished != null && isFinished.getAsBoolean();
    }

    void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    /** Returns whether a tree of Actions contains a lifecycle Action, which makes its leases exclusive. */
    static boolean present(Action action) {
        if (action == null) return false;
        if (action instanceof LifecycleAction) return true;
        if (action instanceof Actions.Parallel parallel) return anyPresent(parallel.Actions());
        if (action instanceof Actions.Race race) return anyPresent(race.Actions());
        if (action instanceof Actions.Deadline deadline) return anyPresent(deadline.Actions());
        if (action instanceof Actions.Sequence sequence) {
            for (Actions.SequenceStep step : sequence.steps()) if (present(step.action())) return true;
            return present(sequence.next());
        }
        if (action instanceof Actions.Cycle cycle) {
            for (Actions.CycleStep step : cycle.steps()) if (present(step.action())) return true;
            return false;
        }
        if (action instanceof Actions.Choice choice) return present(choice.active()) || present(choice.inactive());
        if (action instanceof Actions.WhenBranch branch) return present(branch.active());
        if (action instanceof Actions.Timeout timeout) return present(timeout.action());
        if (action instanceof Actions.WithinTolerance within) return present(within.action());
        if (action instanceof Actions.Interruptible policy) return present(policy.action());
        if (action instanceof Actions.Conditional conditional) {
            return present(conditional.action()) || present(conditional.next());
        }
        if (action instanceof Action.Conditional conditional) {
            return present(conditional.action()) || present(conditional.next());
        }
        if (action instanceof Actions.Then then) return present(then.action()) || present(then.next());
        if (action instanceof Action.Then then) return present(then.action()) || present(then.next());
        if (action instanceof PathAction path) {
            return path.markers().values().stream().anyMatch(LifecycleAction::present);
        }
        return false;
    }

    private static boolean anyPresent(java.util.Collection<Action> actions) {
        for (Action action : actions) if (present(action)) return true;
        return false;
    }

    /** Fluent builder returned by {@link Actions#lifecycle(String)}. */
    public static final class Builder {
        private final String name;
        private Runnable onStart;
        private Runnable onExecute;
        private BooleanSupplier isFinished;
        private Consumer<Boolean> onEnd;
        private final Set<Object> requirements = new LinkedHashSet<>();

        private Builder(String name) {
            this.name = name;
        }

        /** Sets the callback run once when the Action starts. */
        public Builder onStart(Runnable action) {
            onStart = action;
            return this;
        }

        /** Sets the callback run every cycle while the Action runs. */
        public Builder execute(Runnable action) {
            onExecute = action;
            return this;
        }

        /** Sets the finish condition, checked after each execute. Without one the Action never finishes. */
        public Builder until(BooleanSupplier condition) {
            isFinished = Objects.requireNonNull(condition, "condition");
            return this;
        }

        /** Sets the end callback, receiving {@code true} when interrupted and {@code false} when finished. */
        public Builder onEnd(Consumer<Boolean> action) {
            onEnd = action;
            return this;
        }

        /** Sets an end callback that does not need to know how the Action ended. */
        public Builder onEnd(Runnable action) {
            Objects.requireNonNull(action, "action");
            onEnd = interrupted -> action.run();
            return this;
        }

        /**
         * Adds requirements. Accepts Athena declarations (mechanisms, devices, controls, which determine the
         * owning mechanism and its resources), {@link NamedResource}s, and strings, which become named
         * resources.
         */
        public Builder requires(Object... requirements) {
            if (requirements == null) return this;
            for (Object requirement : requirements) {
                Objects.requireNonNull(requirement, "requirement");
                if (requirement instanceof String text) {
                    this.requirements.add(NamedResource.of(text));
                } else {
                    this.requirements.add(requirement);
                }
            }
            return this;
        }

        /** Builds the lifecycle Action. */
        public LifecycleAction build() {
            return new LifecycleAction(name, onStart, onExecute, isFinished, onEnd, requirements);
        }
    }
}
