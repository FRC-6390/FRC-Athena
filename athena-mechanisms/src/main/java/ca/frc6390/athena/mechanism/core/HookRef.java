package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionRef;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Declarative binding from an event to one or more actions.
 *
 * @param event event source
 * @param bindings trigger/action bindings
 */
public record HookRef(EventRef event, List<HookBinding> bindings) {
    public HookRef {
        Objects.requireNonNull(event, "event");
        bindings = Collections.unmodifiableList(new ArrayList<>(
                bindings == null ? List.of() : bindings));
    }

    /**
     * Adds an action that runs when the event starts.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onStart(Runnable action) {
        return onStart(ActionRef.run(action));
    }

    /**
     * Adds a context action that runs when the event starts.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onStart(ActionRef action) {
        return with(HookTrigger.ON_START, action);
    }

    /**
     * Adds an action that runs every tick while the event is active.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef whileActive(Runnable action) {
        return whileActive(ActionRef.run(action));
    }

    /**
     * Adds a context action that runs every tick while the event is active.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef whileActive(ActionRef action) {
        return with(HookTrigger.WHILE_ACTIVE, action);
    }

    /**
     * Adds an action that runs when the event ends.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onEnd(Runnable action) {
        return onEnd(ActionRef.run(action));
    }

    /**
     * Adds a context action that runs when the event ends.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onEnd(ActionRef action) {
        return with(HookTrigger.ON_END, action);
    }

    /**
     * Adds an action that runs when the event becomes inactive.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onInactive(Runnable action) {
        return onInactive(ActionRef.run(action));
    }

    /**
     * Adds a context action that runs when the event becomes inactive.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef onInactive(ActionRef action) {
        return with(HookTrigger.ON_INACTIVE, action);
    }

    /**
     * Adds an action that runs every tick while the event is inactive.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef whileInactive(Runnable action) {
        return whileInactive(ActionRef.run(action));
    }

    /**
     * Adds a context action that runs every tick while the event is inactive.
     *
     * @param action action to run
     * @return updated hook
     */
    public HookRef whileInactive(ActionRef action) {
        return with(HookTrigger.WHILE_INACTIVE, action);
    }

    private HookRef with(HookTrigger trigger, ActionRef action) {
        Objects.requireNonNull(action, "action");
        List<HookBinding> next = new ArrayList<>(bindings);
        next.add(new HookBinding(trigger, action));
        return new HookRef(event, next);
    }

    public record HookBinding(HookTrigger trigger, ActionRef action) {
        public HookBinding {
            Objects.requireNonNull(trigger, "trigger");
            Objects.requireNonNull(action, "action");
        }
    }
}
