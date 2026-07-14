package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Declarative binding from an event to one or more actions.
 */
public record HookBinding(EventBinding event, List<HookAction> actions, boolean isDisabled) {
    public HookBinding(EventBinding event) {
        this(event, List.of(), false);
    }

    public HookBinding(EventBinding event, List<HookAction> actions) {
        this(event, actions, false);
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

    public HookBinding onStart(DeviceAction action) {
        return onStart((ActionBinding) action);
    }

    public HookBinding onStart(Action action) {
        return onStart((ActionBinding) action);
    }

    public HookBinding whileActive(Runnable action) {
        return whileActive(ActionBinding.run(action));
    }

    public HookBinding whileActive(ActionBinding action) {
        return with(Phase.WHILE_ACTIVE, action);
    }

    public HookBinding whileActive(DeviceAction action) {
        return whileActive((ActionBinding) action);
    }

    public HookBinding whileActive(Action action) {
        return whileActive((ActionBinding) action);
    }

    public HookBinding onEnd(Runnable action) {
        return onEnd(ActionBinding.run(action));
    }

    public HookBinding onEnd(ActionBinding action) {
        return with(Phase.ON_END, action);
    }

    public HookBinding onEnd(DeviceAction action) {
        return onEnd((ActionBinding) action);
    }

    public HookBinding onEnd(Action action) {
        return onEnd((ActionBinding) action);
    }

    public HookBinding onInactive(Runnable action) {
        return onInactive(ActionBinding.run(action));
    }

    public HookBinding onInactive(ActionBinding action) {
        return with(Phase.ON_INACTIVE, action);
    }

    public HookBinding onInactive(DeviceAction action) {
        return onInactive((ActionBinding) action);
    }

    public HookBinding onInactive(Action action) {
        return onInactive((ActionBinding) action);
    }

    public HookBinding whileInactive(Runnable action) {
        return whileInactive(ActionBinding.run(action));
    }

    public HookBinding whileInactive(ActionBinding action) {
        return with(Phase.WHILE_INACTIVE, action);
    }

    public HookBinding whileInactive(DeviceAction action) {
        return whileInactive((ActionBinding) action);
    }

    public HookBinding whileInactive(Action action) {
        return whileInactive((ActionBinding) action);
    }

    private HookBinding with(Phase phase, ActionBinding action) {
        Objects.requireNonNull(action, "action");
        List<HookAction> next = new ArrayList<>(actions);
        next.add(new HookAction(phase, action));
        return new HookBinding(event, next, isDisabled);
    }

    public HookBinding disabled() {
        return disabled(true);
    }

    public HookBinding disabled(boolean disabled) {
        return new HookBinding(event, actions, disabled);
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
