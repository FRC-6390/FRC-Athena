package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Declarative binding from an event to one or more actions.
 */
public record HookBinding(EventBinding event, List<HookAction> actions) {
    public HookBinding(EventBinding event) {
        this(event, List.of());
    }

    public HookBinding {
        Objects.requireNonNull(event, "event");
        actions = actions == null ? List.of() : List.copyOf(actions);
    }

    public HookBinding onStart(Runnable action) {
        return onStart(ActionBinding.run(action));
    }

    public HookBinding onStart(ActionBinding action) {
        return with(Phase.ON_START, action);
    }

    public HookBinding whileActive(Runnable action) {
        return whileActive(ActionBinding.run(action));
    }

    public HookBinding whileActive(ActionBinding action) {
        return with(Phase.WHILE_ACTIVE, action);
    }

    public HookBinding onEnd(Runnable action) {
        return onEnd(ActionBinding.run(action));
    }

    public HookBinding onEnd(ActionBinding action) {
        return with(Phase.ON_END, action);
    }

    public HookBinding onInactive(Runnable action) {
        return onInactive(ActionBinding.run(action));
    }

    public HookBinding onInactive(ActionBinding action) {
        return with(Phase.ON_INACTIVE, action);
    }

    public HookBinding whileInactive(Runnable action) {
        return whileInactive(ActionBinding.run(action));
    }

    public HookBinding whileInactive(ActionBinding action) {
        return with(Phase.WHILE_INACTIVE, action);
    }

    private HookBinding with(Phase phase, ActionBinding action) {
        Objects.requireNonNull(action, "action");
        List<HookAction> next = new ArrayList<>(actions);
        next.add(new HookAction(phase, action));
        return new HookBinding(event, next);
    }

    public enum Phase {
        ON_START,
        WHILE_ACTIVE,
        ON_END,
        ON_INACTIVE,
        WHILE_INACTIVE;

        boolean shouldRun(EventBinding event, boolean wasActive, boolean active) {
            return switch (this) {
                case ON_START -> active && (!wasActive || event.pulse());
                case WHILE_ACTIVE -> active;
                case ON_END -> wasActive && !active;
                case ON_INACTIVE -> !active && wasActive;
                case WHILE_INACTIVE -> !active;
            };
        }
    }

    public record HookAction(Phase phase, ActionBinding action) {
        public HookAction {
            Objects.requireNonNull(phase, "phase");
            Objects.requireNonNull(action, "action");
        }
    }
}
