package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.BooleanRef;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.ActionRef;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Factories for event hooks.
 */
public final class Hooks {
    public static final LifecycleHooks robot = lifecycle("robot", LifecycleMode.ROBOT);
    public static final LifecycleHooks disabled = lifecycle("disabled", LifecycleMode.DISABLED);
    public static final LifecycleHooks disable = disabled;
    public static final LifecycleHooks autonomous = lifecycle("autonomous", LifecycleMode.AUTONOMOUS);
    public static final LifecycleHooks auto = autonomous;
    public static final LifecycleHooks teleop = lifecycle("teleop", LifecycleMode.TELEOP);
    public static final LifecycleHooks test = lifecycle("test", LifecycleMode.TEST);
    public static final LifecycleHooks simulation = lifecycle("simulation", LifecycleMode.SIMULATION);
    public static final LifecycleHooks sim = simulation;

    private Hooks() {
    }

    /**
     * Creates a hook from an event.
     *
     * @param event event source
     * @return hook
     */
    public static HookRef when(EventRef event) {
        return new HookRef(Objects.requireNonNull(event, "event"), java.util.List.of());
    }

    /**
     * Creates a level hook from a boolean supplier.
     *
     * @param source source supplier
     * @return hook
     */
    public static HookRef when(BooleanSupplier source) {
        return when(Events.when(source).active());
    }

    /**
     * Creates a level hook from a digital input.
     *
     * @param input digital input
     * @return hook
     */
    public static HookRef when(DigitalInputRef input) {
        return when(Events.when(input).active());
    }

    /**
     * Creates a level hook from a boolean ref.
     *
     * @param signal boolean signal
     * @return hook
     */
    public static HookRef when(BooleanRef signal) {
        return when(Events.when(signal).active());
    }

    public static LifecycleHooks robot() {
        return robot;
    }

    public static LifecycleHooks disabled() {
        return disabled;
    }

    public static LifecycleHooks disable() {
        return disable;
    }

    public static LifecycleHooks autonomous() {
        return autonomous;
    }

    public static LifecycleHooks auto() {
        return auto;
    }

    public static LifecycleHooks teleop() {
        return teleop;
    }

    public static LifecycleHooks test() {
        return test;
    }

    public static LifecycleHooks simulation() {
        return simulation;
    }

    public static LifecycleHooks sim() {
        return sim;
    }

    private static LifecycleHooks lifecycle(String name, LifecycleMode mode) {
        return new LifecycleHooks(name, mode);
    }

    /**
     * Lifecycle mode hook builder.
     */
    public record LifecycleHooks(String name, LifecycleMode mode) {
        public LifecycleHooks {
            name = name == null || name.isBlank() ? "lifecycle" : name;
            mode = mode == null ? LifecycleMode.ROBOT : mode;
        }

        public HookRef onStart(Runnable action) {
            return onStart(ActionRef.run(action));
        }

        public HookRef onStart(ActionRef action) {
            return hook(LifecyclePhase.INIT, action);
        }

        public HookRef whileActive(Runnable action) {
            return whileActive(ActionRef.run(action));
        }

        public HookRef whileActive(ActionRef action) {
            return hook(LifecyclePhase.PERIODIC, action);
        }

        public HookRef onEnd(Runnable action) {
            return onEnd(ActionRef.run(action));
        }

        public HookRef onEnd(ActionRef action) {
            return hook(LifecyclePhase.EXIT, action);
        }

        private HookRef hook(LifecyclePhase phase, ActionRef action) {
            Objects.requireNonNull(action, "action");
            EventRef event = Events.lifecycle(name + eventSuffix(phase), mode, phase);
            return Hooks.when(event).onStart(action);
        }

        private static String eventSuffix(LifecyclePhase phase) {
            return switch (phase) {
                case INIT -> "Init";
                case PERIODIC -> "Periodic";
                case EXIT -> "Exit";
            };
        }
    }
}
