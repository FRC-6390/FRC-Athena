package ca.frc6390.athena.mechanisms;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Runtime-owned state machine configuration used by {@link StatefulMechanism.StatefulMechanismCore}.
 *
 * <p>This exists so the runtime no longer needs to store builder-specific nested binding types
 * directly.</p>
 */
public final class StatefulMechanismRuntimeConfig<T extends Mechanism, E> {
    @FunctionalInterface
    public interface StateHook<T extends Mechanism, E>
            extends java.util.function.Consumer<MechanismContext<T, E>> {
        default void apply(MechanismContext<T, E> context) {
            accept(context);
        }
    }

    @FunctionalInterface
    public interface TransitionHook<T extends Mechanism, E> {
        void apply(MechanismContext<T, E> context, E from, E to);
    }

    @FunctionalInterface
    public interface StateTrigger<T extends Mechanism, E>
            extends Predicate<MechanismContext<T, E>> {
        default boolean shouldQueue(MechanismContext<T, E> context) {
            return test(context);
        }
    }

    public record TransitionHookBinding<T extends Mechanism, E>(
            Object from,
            Object to,
            TransitionHook<T, E> hook) {
        public TransitionHookBinding {
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            Objects.requireNonNull(hook, "hook");
        }
    }

    public record StateTriggerBinding<T extends Mechanism, E>(
            Object state,
            StateTrigger<T, E> trigger) {
        public StateTriggerBinding {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(trigger, "trigger");
        }
    }

    private final double delay;
    private final Map<Object, Function<T, Boolean>> stateActions;
    private final Map<Object, List<StateHook<T, E>>> enterStateHooks;
    private final Map<Object, List<StateHook<T, E>>> stateHooks;
    private final Map<Object, List<StateHook<T, E>>> exitStateHooks;
    private final List<TransitionHookBinding<T, E>> transitionHooks;
    private final List<StateHook<T, E>> alwaysHooks;
    private final List<StateHook<T, E>> exitAlwaysHooks;
    private final Map<String, BooleanSupplier> inputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;
    private final List<StateTriggerBinding<T, E>> stateTriggers;

    public StatefulMechanismRuntimeConfig(
            double delay,
            Map<Object, Function<T, Boolean>> stateActions,
            Map<Object, List<StateHook<T, E>>> enterStateHooks,
            Map<Object, List<StateHook<T, E>>> stateHooks,
            Map<Object, List<StateHook<T, E>>> exitStateHooks,
            List<TransitionHookBinding<T, E>> transitionHooks,
            List<StateHook<T, E>> alwaysHooks,
            List<StateHook<T, E>> exitAlwaysHooks,
            Map<String, BooleanSupplier> inputs,
            Map<String, DoubleSupplier> doubleInputs,
            Map<String, IntSupplier> intInputs,
            Map<String, Supplier<String>> stringInputs,
            Map<String, Supplier<Pose2d>> pose2dInputs,
            Map<String, Supplier<Pose3d>> pose3dInputs,
            Map<String, Supplier<?>> objectInputs,
            List<StateTriggerBinding<T, E>> stateTriggers) {
        this.delay = delay;
        this.stateActions = Map.copyOf(Objects.requireNonNull(stateActions, "stateActions"));
        this.enterStateHooks = copyHookMap(enterStateHooks, "enterStateHooks");
        this.stateHooks = copyHookMap(stateHooks, "stateHooks");
        this.exitStateHooks = copyHookMap(exitStateHooks, "exitStateHooks");
        this.transitionHooks = List.copyOf(Objects.requireNonNull(transitionHooks, "transitionHooks"));
        this.alwaysHooks = List.copyOf(Objects.requireNonNull(alwaysHooks, "alwaysHooks"));
        this.exitAlwaysHooks = List.copyOf(Objects.requireNonNull(exitAlwaysHooks, "exitAlwaysHooks"));
        this.inputs = Map.copyOf(Objects.requireNonNull(inputs, "inputs"));
        this.doubleInputs = Map.copyOf(Objects.requireNonNull(doubleInputs, "doubleInputs"));
        this.intInputs = Map.copyOf(Objects.requireNonNull(intInputs, "intInputs"));
        this.stringInputs = Map.copyOf(Objects.requireNonNull(stringInputs, "stringInputs"));
        this.pose2dInputs = Map.copyOf(Objects.requireNonNull(pose2dInputs, "pose2dInputs"));
        this.pose3dInputs = Map.copyOf(Objects.requireNonNull(pose3dInputs, "pose3dInputs"));
        this.objectInputs = Map.copyOf(Objects.requireNonNull(objectInputs, "objectInputs"));
        this.stateTriggers = List.copyOf(Objects.requireNonNull(stateTriggers, "stateTriggers"));
    }

    public static <T extends Mechanism, E> StatefulMechanismRuntimeConfig<T, E> empty() {
        return new StatefulMechanismRuntimeConfig<>(
                0.0,
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                List.of(),
                List.of(),
                List.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                List.of());
    }

    public double delay() {
        return delay;
    }

    public Map<Object, Function<T, Boolean>> stateActions() {
        return stateActions;
    }

    public Map<Object, List<StateHook<T, E>>> enterStateHooks() {
        return enterStateHooks;
    }

    public Map<Object, List<StateHook<T, E>>> stateHooks() {
        return stateHooks;
    }

    public Map<Object, List<StateHook<T, E>>> exitStateHooks() {
        return exitStateHooks;
    }

    public List<TransitionHookBinding<T, E>> transitionHooks() {
        return transitionHooks;
    }

    public List<StateHook<T, E>> alwaysHooks() {
        return alwaysHooks;
    }

    public List<StateHook<T, E>> exitAlwaysHooks() {
        return exitAlwaysHooks;
    }

    public Map<String, BooleanSupplier> inputs() {
        return inputs;
    }

    public Map<String, DoubleSupplier> doubleInputs() {
        return doubleInputs;
    }

    public Map<String, IntSupplier> intInputs() {
        return intInputs;
    }

    public Map<String, Supplier<String>> stringInputs() {
        return stringInputs;
    }

    public Map<String, Supplier<Pose2d>> pose2dInputs() {
        return pose2dInputs;
    }

    public Map<String, Supplier<Pose3d>> pose3dInputs() {
        return pose3dInputs;
    }

    public Map<String, Supplier<?>> objectInputs() {
        return objectInputs;
    }

    public List<StateTriggerBinding<T, E>> stateTriggers() {
        return stateTriggers;
    }

    private static <T extends Mechanism, E>
            Map<Object, List<StateHook<T, E>>> copyHookMap(
                    Map<Object, List<StateHook<T, E>>> hooks,
                    String name) {
        Objects.requireNonNull(hooks, name);
        if (hooks.isEmpty()) {
            return Map.of();
        }
        Map<Object, List<StateHook<T, E>>> copy = new LinkedHashMap<>();
        hooks.forEach((state, values) -> {
            if (state == null || values == null || values.isEmpty()) {
                return;
            }
            copy.put(state, List.copyOf(values));
        });
        return Map.copyOf(copy);
    }
}
