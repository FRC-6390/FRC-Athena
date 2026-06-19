package ca.frc6390.athena.mechanisms;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotCoreHooks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Runtime-owned superstructure configuration used by {@link SuperstructureMechanism}.
 *
 * <p>Like {@link StatefulMechanismRuntimeConfig}, this isolates runtime storage from the authoring
 * surface.</p>
 */
public final class SuperstructureRuntimeConfig<S, SP> {
    @FunctionalInterface
    public interface Binding<SP> extends Consumer<SuperstructureContext<SP>> {
        default void apply(SuperstructureContext<SP> context) {
            accept(context);
        }
    }

    @FunctionalInterface
    public interface TransitionHook<SP, S> {
        void apply(SuperstructureContext<SP> context, S from, S to);
    }

    public record TransitionBinding<SP, S>(
            S from,
            S to,
            TransitionHook<SP, S> hook) {
        public TransitionBinding {
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            Objects.requireNonNull(hook, "hook");
        }
    }

    public record LifecycleHookBinding<SP, S>(
            Binding<SP> binding,
            List<S> states) {
        public LifecycleHookBinding {
            Objects.requireNonNull(binding, "binding");
            states = states == null ? List.of() : List.copyOf(states);
        }

        public boolean appliesTo(S activeState) {
            if (states == null || states.isEmpty()) {
                return true;
            }
            if (activeState == null) {
                return false;
            }
            for (S candidate : states) {
                if (candidate == activeState || candidate.equals(activeState)) {
                    return true;
                }
            }
            return false;
        }
    }

    public static final class Constraint<S, SP> {
        final Predicate<SuperstructureContext<SP>> guard;
        final List<S> transitionStates;

        public Constraint(Predicate<SuperstructureContext<SP>> guard, List<S> transitionStates) {
            this.guard = Objects.requireNonNull(guard, "guard");
            this.transitionStates = List.copyOf(Objects.requireNonNull(transitionStates, "transitionStates"));
        }
    }

    static final class Attachment<SP, E> {
        final Function<SP, E> childMapper;
        final Function<SuperstructureContext<SP>, Mechanism> resolver;
        final Function<SuperstructureContext<SP>, Pose3d> poseSupplier;

        Attachment(
                Function<SP, E> childMapper,
                Function<SuperstructureContext<SP>, Mechanism> resolver,
                Function<SuperstructureContext<SP>, Pose3d> poseSupplier) {
            this.childMapper = childMapper;
            this.resolver = resolver;
            this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
        }
    }

    private final S initialState;
    private final double stateMachineDelaySeconds;
    private final List<SuperstructureMechanism.Child<SP, ?>> children;
    private final Map<S, Constraint<S, SP>> constraints;
    private final List<Attachment<SP, ?>> attachments;
    private final Map<String, BooleanSupplier> inputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;
    private final Map<S, List<Binding<SP>>> enterBindings;
    private final List<TransitionBinding<SP, S>> transitionBindings;
    private final Map<S, List<Binding<SP>>> bindings;
    private final List<Binding<SP>> alwaysBindings;
    private final List<Binding<SP>> periodicBindings;
    private final Map<S, List<Binding<SP>>> exitBindings;
    private final List<Binding<SP>> exitAlwaysBindings;
    private final Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> lifecycleBindings;
    private final List<LifecycleHookBinding<SP, S>> lifecycleExitBindings;

    public SuperstructureRuntimeConfig(
            S initialState,
            double stateMachineDelaySeconds,
            List<SuperstructureMechanism.Child<SP, ?>> children,
            Map<S, Constraint<S, SP>> constraints,
            List<Attachment<SP, ?>> attachments,
            Map<String, BooleanSupplier> inputs,
            Map<String, DoubleSupplier> doubleInputs,
            Map<String, IntSupplier> intInputs,
            Map<String, Supplier<String>> stringInputs,
            Map<String, Supplier<Pose2d>> pose2dInputs,
            Map<String, Supplier<Pose3d>> pose3dInputs,
            Map<String, Supplier<?>> objectInputs,
            Map<S, List<Binding<SP>>> enterBindings,
            List<TransitionBinding<SP, S>> transitionBindings,
            Map<S, List<Binding<SP>>> bindings,
            List<Binding<SP>> alwaysBindings,
            List<Binding<SP>> periodicBindings,
            Map<S, List<Binding<SP>>> exitBindings,
            List<Binding<SP>> exitAlwaysBindings,
            Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> lifecycleBindings,
            List<LifecycleHookBinding<SP, S>> lifecycleExitBindings) {
        this.initialState = Objects.requireNonNull(initialState, "initialState");
        this.stateMachineDelaySeconds = stateMachineDelaySeconds;
        this.children = List.copyOf(Objects.requireNonNull(children, "children"));
        this.constraints = Map.copyOf(Objects.requireNonNull(constraints, "constraints"));
        this.attachments = List.copyOf(Objects.requireNonNull(attachments, "attachments"));
        this.inputs = Map.copyOf(Objects.requireNonNull(inputs, "inputs"));
        this.doubleInputs = Map.copyOf(Objects.requireNonNull(doubleInputs, "doubleInputs"));
        this.intInputs = Map.copyOf(Objects.requireNonNull(intInputs, "intInputs"));
        this.stringInputs = Map.copyOf(Objects.requireNonNull(stringInputs, "stringInputs"));
        this.pose2dInputs = Map.copyOf(Objects.requireNonNull(pose2dInputs, "pose2dInputs"));
        this.pose3dInputs = Map.copyOf(Objects.requireNonNull(pose3dInputs, "pose3dInputs"));
        this.objectInputs = Map.copyOf(Objects.requireNonNull(objectInputs, "objectInputs"));
        this.enterBindings = copyBindingMap(enterBindings, "enterBindings");
        this.transitionBindings = List.copyOf(Objects.requireNonNull(transitionBindings, "transitionBindings"));
        this.bindings = copyBindingMap(bindings, "bindings");
        this.alwaysBindings = List.copyOf(Objects.requireNonNull(alwaysBindings, "alwaysBindings"));
        this.periodicBindings = List.copyOf(Objects.requireNonNull(periodicBindings, "periodicBindings"));
        this.exitBindings = copyBindingMap(exitBindings, "exitBindings");
        this.exitAlwaysBindings = List.copyOf(Objects.requireNonNull(exitAlwaysBindings, "exitAlwaysBindings"));
        this.lifecycleBindings = copyLifecycleBindingMap(lifecycleBindings, "lifecycleBindings");
        this.lifecycleExitBindings = List.copyOf(Objects.requireNonNull(lifecycleExitBindings, "lifecycleExitBindings"));
    }

    public S initialState() {
        return initialState;
    }

    public double stateMachineDelaySeconds() {
        return stateMachineDelaySeconds;
    }

    public List<SuperstructureMechanism.Child<SP, ?>> children() {
        return children;
    }

    public Map<S, Constraint<S, SP>> constraints() {
        return constraints;
    }

    public List<Attachment<SP, ?>> attachments() {
        return attachments;
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

    public Map<S, List<Binding<SP>>> enterBindings() {
        return enterBindings;
    }

    public List<TransitionBinding<SP, S>> transitionBindings() {
        return transitionBindings;
    }

    public Map<S, List<Binding<SP>>> bindings() {
        return bindings;
    }

    public List<Binding<SP>> alwaysBindings() {
        return alwaysBindings;
    }

    public List<Binding<SP>> periodicBindings() {
        return periodicBindings;
    }

    public Map<S, List<Binding<SP>>> exitBindings() {
        return exitBindings;
    }

    public List<Binding<SP>> exitAlwaysBindings() {
        return exitAlwaysBindings;
    }

    public Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> lifecycleBindings() {
        return lifecycleBindings;
    }

    public List<LifecycleHookBinding<SP, S>> lifecycleExitBindings() {
        return lifecycleExitBindings;
    }

    private static <S, SP>
            Map<S, List<Binding<SP>>> copyBindingMap(Map<S, List<Binding<SP>>> bindings, String name) {
        Objects.requireNonNull(bindings, name);
        if (bindings.isEmpty()) {
            return Map.of();
        }
        Map<S, List<Binding<SP>>> copy = new LinkedHashMap<>();
        bindings.forEach((state, values) -> {
            if (state == null || values == null || values.isEmpty()) {
                return;
            }
            copy.put(state, List.copyOf(values));
        });
        return Map.copyOf(copy);
    }

    private static <S, SP>
            Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> copyLifecycleBindingMap(
                    Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> bindings,
                    String name) {
        Objects.requireNonNull(bindings, name);
        if (bindings.isEmpty()) {
            return Map.of();
        }
        Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> copy = new LinkedHashMap<>();
        bindings.forEach((phase, values) -> {
            if (phase == null || values == null || values.isEmpty()) {
                return;
            }
            copy.put(phase, List.copyOf(values));
        });
        return Map.copyOf(copy);
    }
}
