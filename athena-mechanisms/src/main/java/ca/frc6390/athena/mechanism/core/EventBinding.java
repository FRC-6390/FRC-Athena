package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.List;

/**
 * Declarative event source for hooks.
 */
public interface EventBinding {
    String name();

    boolean sourceActive(EventContext context);

    default boolean active(EventContext context, boolean previousSourceActive) {
        return sourceActive(context);
    }

    default boolean active(EventContext context, boolean previousSourceActive, boolean currentSourceActive) {
        return currentSourceActive;
    }

    default boolean pulse() {
        return false;
    }

    default void afterRun(EventContext context) {
    }

    default List<Object> declarations() {
        return List.of();
    }

    default HookBinding onStart(Runnable action) {
        return new HookBinding(this).onStart(action);
    }

    default HookBinding onStart(ActionBinding action) {
        return new HookBinding(this).onStart(action);
    }

    default HookBinding onStart(DeviceAction action) {
        return new HookBinding(this).onStart(action);
    }

    default HookBinding onStart(Action action) {
        return new HookBinding(this).onStart(action);
    }

    default HookBinding whileActive(Runnable action) {
        return new HookBinding(this).whileActive(action);
    }

    default HookBinding whileActive(ActionBinding action) {
        return new HookBinding(this).whileActive(action);
    }

    default HookBinding whileActive(DeviceAction action) {
        return new HookBinding(this).whileActive(action);
    }

    default HookBinding whileActive(Action action) {
        return new HookBinding(this).whileActive(action);
    }

    default HookBinding onEnd(Runnable action) {
        return new HookBinding(this).onEnd(action);
    }

    default HookBinding onEnd(ActionBinding action) {
        return new HookBinding(this).onEnd(action);
    }

    default HookBinding onEnd(DeviceAction action) {
        return new HookBinding(this).onEnd(action);
    }

    default HookBinding onEnd(Action action) {
        return new HookBinding(this).onEnd(action);
    }

    default HookBinding onInactive(Runnable action) {
        return new HookBinding(this).onInactive(action);
    }

    default HookBinding onInactive(ActionBinding action) {
        return new HookBinding(this).onInactive(action);
    }

    default HookBinding onInactive(DeviceAction action) {
        return new HookBinding(this).onInactive(action);
    }

    default HookBinding onInactive(Action action) {
        return new HookBinding(this).onInactive(action);
    }

    default HookBinding whileInactive(Runnable action) {
        return new HookBinding(this).whileInactive(action);
    }

    default HookBinding whileInactive(ActionBinding action) {
        return new HookBinding(this).whileInactive(action);
    }

    default HookBinding whileInactive(DeviceAction action) {
        return new HookBinding(this).whileInactive(action);
    }

    default HookBinding whileInactive(Action action) {
        return new HookBinding(this).whileInactive(action);
    }
}
