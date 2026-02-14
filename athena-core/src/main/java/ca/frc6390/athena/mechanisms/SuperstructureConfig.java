package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
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
import ca.frc6390.athena.core.hooks.LifecycleHooksSectionBase;
import ca.frc6390.athena.core.input.TypedInputRegistration;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import ca.frc6390.athena.mechanisms.Mechanism;

/**
 * Declarative config for a superstructure that coordinates multiple stateful mechanisms.
 * Callers map each superstate setpoint to the child mechanism state via a mapper function
 * (for example, {@code mechanisms(m -> m.mechanism(ELEVATOR_CONFIG, SuperTuple::elev))}). Constraints can
 * reference the child mechanisms through {@link SuperstructureContext#mechanisms()} without
 * naming them explicitly, and can optionally insert transition states before a guarded state.
 *
 * @param <S> superstate enum type
 * @param <SP> setpoint tuple returned by the superstate enum
 */
public final class SuperstructureConfig<S extends Enum<S> & SetpointProvider<SP>, SP> {

    private S initialState;
    private String superstructureName;
    private double stateMachineDelaySeconds = 0.0;
    private final List<ChildFactory<SP>> childConfigs = new ArrayList<>();
    private final Map<S, Constraint<S, SP>> constraints = new HashMap<>();
    private final List<Attachment<SP, ?>> attachments = new ArrayList<>();
    private final Map<String, BooleanSupplier> inputs = new HashMap<>();
    private final Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
    private final Map<String, IntSupplier> intInputs = new HashMap<>();
    private final Map<String, Supplier<String>> stringInputs = new HashMap<>();
    private final Map<String, Supplier<Pose2d>> pose2dInputs = new HashMap<>();
    private final Map<String, Supplier<Pose3d>> pose3dInputs = new HashMap<>();
    private final Map<String, Supplier<?>> objectInputs = new HashMap<>();
    private final Map<S, List<Binding<SP>>> enterBindings = new HashMap<>();
    private final List<TransitionBinding<SP, S>> transitionBindings = new ArrayList<>();
    private final Map<S, List<Binding<SP>>> bindings = new HashMap<>();
    private final List<Binding<SP>> alwaysBindings = new ArrayList<>();
    private final List<Binding<SP>> periodicBindings = new ArrayList<>();
    /** Optional hooks that run once during robot init after all mechanisms/superstructures are registered. */
    public List<Binding<SP>> initBindings = new ArrayList<>();
    /** Optional lifecycle hooks keyed by robot phase (RobotCore parity). */
    public Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<SP, S>>> lifecycleBindings = new HashMap<>();
    /** Optional hooks that run whenever a robot mode exits (before mode-specific exit hooks). */
    public List<LifecycleHookBinding<SP, S>> lifecycleExitBindings = new ArrayList<>();
    private final Map<S, List<Binding<SP>>> exitBindings = new HashMap<>();
    private final List<Binding<SP>> exitAlwaysBindings = new ArrayList<>();
    private SP initialSetpoint;

    public SuperstructureConfig(S initialState) {
        this.initialState = initialState;
        this.initialSetpoint = initialState != null ? initialState.getSetpoint() : null;
    }

    public static <S extends Enum<S> & SetpointProvider<SP>, SP> SuperstructureConfig<S, SP> create(S initialState) {
        return new SuperstructureConfig<>(initialState);
    }

    /**
     * Named variant of {@link #create(Enum)}.
     */
    public static <S extends Enum<S> & SetpointProvider<SP>, SP> SuperstructureConfig<S, SP> create(String name, S initialState) {
        return create(initialState).named(name);
    }

    /**
     * Assigns the {@link edu.wpi.first.wpilibj2.command.Subsystem} name of the resulting
     * superstructure. Recommended so RobotCore can enforce uniqueness and so teams can retrieve
     * the owning superstructure from constants.
     */
    public SuperstructureConfig<S, SP> named(String name) {
        this.superstructureName = name;
        return this;
    }

    public String name() {
        return superstructureName;
    }

    public SuperstructureMechanism<S, SP> build() {
        ensureInitialState();
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        SuperstructureMechanism<S, SP> mech = new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                inputs,
                doubleInputs,
                intInputs,
                stringInputs,
                pose2dInputs,
                pose3dInputs,
                objectInputs,
                mergedEnterBindings(),
                List.copyOf(transitionBindings),
                mergedBindings(),
                mergedAlwaysBindings(),
                mergedPeriodicBindings(),
                mergedExitBindings(),
                mergedExitAlwaysBindings());
        if (superstructureName != null && !superstructureName.isBlank()) {
            mech.setName(superstructureName);
        }
        mech.setSourceConfig(this);
        return mech;
    }

    /**
     * Builds a superstructure with additional/overridden inputs supplied at build time.
     */
    public SuperstructureMechanism<S, SP> build(Map<String, BooleanSupplier> inputOverrides) {
        ensureInitialState();
        Map<String, BooleanSupplier> merged = new HashMap<>(inputs);
        merged.putAll(inputOverrides);
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        SuperstructureMechanism<S, SP> mech = new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                merged,
                doubleInputs,
                intInputs,
                stringInputs,
                pose2dInputs,
                pose3dInputs,
                objectInputs,
                mergedEnterBindings(),
                List.copyOf(transitionBindings),
                mergedBindings(),
                mergedAlwaysBindings(),
                mergedPeriodicBindings(),
                mergedExitBindings(),
                mergedExitAlwaysBindings());
        if (superstructureName != null && !superstructureName.isBlank()) {
            mech.setName(superstructureName);
        }
        mech.setSourceConfig(this);
        return mech;
    }

    /**
     * Builds a superstructure with additional/overridden inputs supplied at build time.
     */
    public SuperstructureMechanism<S, SP> build(Map<String, BooleanSupplier> booleanOverrides,
                                               Map<String, DoubleSupplier> doubleOverrides,
                                               Map<String, Supplier<?>> objectOverrides) {
        ensureInitialState();
        Map<String, BooleanSupplier> mergedBooleans = new HashMap<>(inputs);
        if (booleanOverrides != null) {
            mergedBooleans.putAll(booleanOverrides);
        }
        Map<String, DoubleSupplier> mergedDoubles = new HashMap<>(doubleInputs);
        if (doubleOverrides != null) {
            mergedDoubles.putAll(doubleOverrides);
        }
        Map<String, Supplier<?>> mergedObjects = new HashMap<>(objectInputs);
        if (objectOverrides != null) {
            mergedObjects.putAll(objectOverrides);
        }
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        SuperstructureMechanism<S, SP> mech = new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                mergedBooleans,
                mergedDoubles,
                intInputs,
                stringInputs,
                pose2dInputs,
                pose3dInputs,
                mergedObjects,
                mergedEnterBindings(),
                List.copyOf(transitionBindings),
                mergedBindings(),
                mergedAlwaysBindings(),
                mergedPeriodicBindings(),
                mergedExitBindings(),
                mergedExitAlwaysBindings());
        if (superstructureName != null && !superstructureName.isBlank()) {
            mech.setName(superstructureName);
        }
        mech.setSourceConfig(this);
        return mech;
    }

    private void ensureInitialState() {
        this.initialState = Objects.requireNonNull(initialState, "initialState");
        if (initialSetpoint == null) {
            this.initialSetpoint = initialState.getSetpoint();
        }
    }

    /**
     * Sets the state machine delay (seconds).
     */
    public SuperstructureConfig<S, SP> stateMachineDelay(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
        return this;
    }

    /**
     * Sectioned fluent API: mechanisms (leaf mechanisms and nested superstructures).
     */
    public SuperstructureConfig<S, SP> mechanisms(Consumer<MechanismsSection<S, SP>> section) {
        if (section != null) {
            section.accept(new MechanismsSection<>(this));
        }
        return this;
    }

    public static final class MechanismsSection<S extends Enum<S> & SetpointProvider<SP>, SP> {
        private final SuperstructureConfig<S, SP> owner;

        private MechanismsSection(SuperstructureConfig<S, SP> owner) {
            this.owner = owner;
        }

        public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> MechanismsSection<S, SP> mechanism(
                MechanismConfig<T> config,
                Function<SP, E> mapper) {
            Objects.requireNonNull(config, "config");
            Objects.requireNonNull(mapper, "mapper");
            owner.ensureInitialState();
            owner.childConfigs.add(() -> {
                Mechanism mechanism = config.build();
                E sample = mapper.apply(owner.initialSetpoint);
                Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
                return new SuperstructureMechanism.Child<>(mechanism, mapper, stateType);
            });
            return this;
        }

        public <E extends Enum<E> & SetpointProvider<Double>> MechanismsSection<S, SP> existing(
                StatefulMechanism<E> mechanism,
                Function<SP, E> mapper) {
            Objects.requireNonNull(mechanism, "mechanism");
            Objects.requireNonNull(mapper, "mapper");
            owner.ensureInitialState();
            owner.childConfigs.add(() -> {
                E sample = mapper.apply(owner.initialSetpoint);
                Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
                return new SuperstructureMechanism.Child<>(mechanism, mapper, stateType);
            });
            return this;
        }

        public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> MechanismsSection<S, SP> existing(
                T mechanism,
                Function<SP, E> mapper) {
            Objects.requireNonNull(mechanism, "mechanism");
            Objects.requireNonNull(mapper, "mapper");
            owner.ensureInitialState();
            E sample = mapper.apply(owner.initialSetpoint);
            Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
            owner.childConfigs.add(() -> new SuperstructureMechanism.Child<>(mechanism, mapper, stateType));
            return this;
        }

        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> MechanismsSection<S, SP> superstructure(
                SuperstructureConfig<CS, CSP> config,
                Function<SP, CS> mapper) {
            Objects.requireNonNull(config, "config");
            Objects.requireNonNull(mapper, "mapper");
            owner.ensureInitialState();
            CS sample = mapper.apply(owner.initialSetpoint);
            Class<CS> stateType = sample != null ? sample.getDeclaringClass() : null;
            owner.childConfigs.add(() -> {
                SuperstructureMechanism<CS, CSP> superstructure = config.build();
                return new SuperstructureMechanism.Child<>(superstructure, mapper, stateType);
            });
            return this;
        }

        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> MechanismsSection<S, SP> existingSuperstructure(
                SuperstructureMechanism<CS, CSP> superstructure,
                Function<SP, CS> mapper) {
            Objects.requireNonNull(superstructure, "superstructure");
            Objects.requireNonNull(mapper, "mapper");
            owner.ensureInitialState();
            CS sample = mapper.apply(owner.initialSetpoint);
            Class<CS> stateType = sample != null ? sample.getDeclaringClass() : null;
            owner.childConfigs.add(() -> new SuperstructureMechanism.Child<>(superstructure, mapper, stateType));
            return this;
        }
    }

    /**
     * Sectioned fluent API: constraints.
     */
    public SuperstructureConfig<S, SP> constraints(Consumer<ConstraintsSection<S, SP>> section) {
        if (section != null) {
            section.accept(new ConstraintsSection<>(this));
        }
        return this;
    }

    public static final class ConstraintsSection<S extends Enum<S> & SetpointProvider<SP>, SP> {
        private final SuperstructureConfig<S, SP> owner;

        private ConstraintsSection(SuperstructureConfig<S, SP> owner) {
            this.owner = owner;
        }

        @SafeVarargs
        public final ConstraintsSection<S, SP> state(
                S state,
                Predicate<SuperstructureContext<SP>> guard,
                S... transitionStates) {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(guard, "guard");
            List<S> transitions = transitionStates == null ? List.of() : List.of(transitionStates);
            transitions.forEach(t -> Objects.requireNonNull(t, "transitionStates cannot contain null"));
            Constraint<S, SP> constraint = new Constraint<>(guard, transitions);
            owner.constraints.put(state, constraint);
            return this;
        }
    }

    /**
     * Sectioned fluent API: simulation attachments.
     */
    public SuperstructureConfig<S, SP> sim(Consumer<SimulationBuilder<SP>> configurer) {
        Objects.requireNonNull(configurer, "configurer");
        SimulationBuilder<SP> simBuilder = new SimulationBuilder<>();
        configurer.accept(simBuilder);
        attachments.addAll(simBuilder.attachments);
        return this;
    }

    /**
     * Sectioned fluent API: hooks.
     */
    public SuperstructureConfig<S, SP> hooks(Consumer<HooksSection<S, SP>> section) {
        if (section != null) {
            section.accept(new HooksSection<>(this));
        }
        return this;
    }

    public static final class HooksSection<S extends Enum<S> & SetpointProvider<SP>, SP>
            extends LifecycleHooksSectionBase<HooksSection<S, SP>, Binding<SP>, S> {
        private final SuperstructureConfig<S, SP> owner;

        private HooksSection(SuperstructureConfig<S, SP> owner) {
            this.owner = owner;
        }

        @Override
        protected HooksSection<S, SP> self() {
            return this;
        }

        @Override
        protected void addPhaseBinding(RobotCoreHooks.Phase phase, Binding<SP> binding, List<S> states) {
            owner.addLifecycleBinding(phase, binding, states);
        }

        @Override
        protected void addPhaseExitBinding(Binding<SP> binding, List<S> states) {
            owner.addLifecycleExitBinding(binding, states);
        }

        @SafeVarargs
        public final HooksSection<S, SP> onStatePeriodic(Binding<SP> binding, S... states) {
            Objects.requireNonNull(binding, "binding");
            if (states == null || states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one superstate; use always(binding) for always-on hooks");
            }
            if (Arrays.stream(states).anyMatch(Objects::isNull)) {
                throw new IllegalArgumentException("states must not contain null entries");
            }
            for (S state : states) {
                owner.bindings.computeIfAbsent(state, key -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        public HooksSection<S, SP> always(Binding<SP> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.alwaysBindings.add(binding);
            return this;
        }

        @SafeVarargs
        public final HooksSection<S, SP> onStateExit(Binding<SP> binding, S... states) {
            Objects.requireNonNull(binding, "binding");
            if (states == null || states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one superstate; use onAnyStateExit(binding) for any-state exit hooks");
            }
            if (Arrays.stream(states).anyMatch(Objects::isNull)) {
                throw new IllegalArgumentException("states must not contain null entries");
            }
            for (S state : states) {
                owner.exitBindings.computeIfAbsent(state, key -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        public HooksSection<S, SP> onAnyStateExit(Binding<SP> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.exitAlwaysBindings.add(binding);
            return this;
        }

        @SafeVarargs
        public final HooksSection<S, SP> onStateEnter(Binding<SP> binding, S... states) {
            Objects.requireNonNull(binding, "binding");
            if (states == null || states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one superstate");
            }
            if (Arrays.stream(states).anyMatch(Objects::isNull)) {
                throw new IllegalArgumentException("states must not contain null entries");
            }
            for (S state : states) {
                owner.enterBindings.computeIfAbsent(state, key -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        public HooksSection<S, SP> onStateTransition(TransitionHook<SP, S> hook, S from, S to) {
            Objects.requireNonNull(hook, "hook");
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            owner.transitionBindings.add(new TransitionBinding<>(from, to, hook));
            return this;
        }

        @SafeVarargs
        public final HooksSection<S, SP> onStateTransition(TransitionHook<SP, S> hook, StateTransitionPair<S>... pairs) {
            Objects.requireNonNull(hook, "hook");
            Objects.requireNonNull(pairs, "pairs");
            for (StateTransitionPair<S> pair : pairs) {
                if (pair == null || pair.from == null || pair.to == null) {
                    continue;
                }
                owner.transitionBindings.add(new TransitionBinding<>(pair.from, pair.to, hook));
            }
            return this;
        }

        public HooksSection<S, SP> onRobotPeriodic(Binding<SP> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.periodicBindings.add(binding);
            return this;
        }

        @SafeVarargs
        public final HooksSection<S, SP> onRobotPeriodic(Binding<SP> binding, S... states) {
            return onPeriodic(binding, states);
        }

        /**
         * Registers a hook that runs once during {@code RobotCore.robotInit()} after all mechanisms
         * (and superstructures) have been registered with the {@code RobotCore}.
         */
        public HooksSection<S, SP> onInit(Binding<SP> binding) {
            owner.addLifecycleBinding(RobotCoreHooks.Phase.ROBOT_INIT, binding, List.of());
            return this;
        }
    }

    final void addLifecycleBinding(RobotCoreHooks.Phase phase, Binding<SP> binding, List<S> states) {
        Objects.requireNonNull(phase, "phase");
        Objects.requireNonNull(binding, "binding");
        lifecycleBindings
                .computeIfAbsent(phase, unused -> new ArrayList<>())
                .add(new LifecycleHookBinding<>(binding, sanitizeStates(states)));
    }

    final void addLifecycleExitBinding(Binding<SP> binding, List<S> states) {
        Objects.requireNonNull(binding, "binding");
        lifecycleExitBindings.add(new LifecycleHookBinding<>(binding, sanitizeStates(states)));
    }

    private List<S> sanitizeStates(List<S> states) {
        if (states == null || states.isEmpty()) {
            return List.of();
        }
        List<S> copied = new ArrayList<>(states.size());
        for (S state : states) {
            if (state != null) {
                copied.add(state);
            }
        }
        return copied;
    }

    void runInitHooks(SuperstructureContext<SP> context, S activeState) {
        runLegacyInitHooks(context);
        runPhaseHooks(context, activeState, RobotCoreHooks.Phase.ROBOT_INIT);
    }

    void runPhaseHooks(SuperstructureContext<SP> context, S activeState, RobotCoreHooks.Phase phase) {
        if (context == null || phase == null) {
            return;
        }
        if (isExitPhase(phase)) {
            runLifecycleHooks(context, activeState, lifecycleExitBindings);
        }
        runLifecycleHooks(context, activeState, lifecycleBindings.get(phase));
    }

    private void runLegacyInitHooks(SuperstructureContext<SP> context) {
        if (context == null || initBindings == null || initBindings.isEmpty()) {
            return;
        }
        for (Binding<SP> binding : initBindings) {
            if (binding != null) {
                binding.apply(context);
            }
        }
    }

    private void runLifecycleHooks(
            SuperstructureContext<SP> context,
            S activeState,
            List<LifecycleHookBinding<SP, S>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return;
        }
        for (LifecycleHookBinding<SP, S> hook : bindings) {
            if (hook == null || hook.binding() == null || !hook.appliesTo(activeState)) {
                continue;
            }
            hook.binding().apply(context);
        }
    }

    private static boolean isExitPhase(RobotCoreHooks.Phase phase) {
        return phase == RobotCoreHooks.Phase.DISABLED_EXIT
                || phase == RobotCoreHooks.Phase.TELEOP_EXIT
                || phase == RobotCoreHooks.Phase.AUTONOMOUS_EXIT
                || phase == RobotCoreHooks.Phase.TEST_EXIT;
    }

    /**
     * Sectioned fluent API: inputs (typed external values usable by hooks and constraints).
     */
    public SuperstructureConfig<S, SP> inputs(Consumer<InputsSection<S, SP>> section) {
        if (section != null) {
            section.accept(new InputsSection<>(this));
        }
        return this;
    }

    public static final class InputsSection<S extends Enum<S> & SetpointProvider<SP>, SP> {
        private final SuperstructureConfig<S, SP> owner;

        private InputsSection(SuperstructureConfig<S, SP> owner) {
            this.owner = owner;
        }

        public InputsSection<S, SP> boolVal(String key, BooleanSupplier supplier) {
            TypedInputRegistration.put(owner.inputs, key, supplier);
            return this;
        }

        /**
         * Convenience overload for limit-switch inputs.
         */
        public InputsSection<S, SP> boolVal(String key, GenericLimitSwitchConfig config) {
            Objects.requireNonNull(config, "config");
            TypedInputRegistration.put(owner.inputs, key, config.name(key).toSupplier());
            return this;
        }

        /**
         * Convenience overload for limit-switch inputs.
         */
        public InputsSection<S, SP> boolVal(GenericLimitSwitchConfig config) {
            Objects.requireNonNull(config, "config");
            String key = config.name() != null ? config.name() : ("dio-" + config.id());
            TypedInputRegistration.put(owner.inputs, key, config.toSupplier());
            return this;
        }

        public InputsSection<S, SP> doubleVal(String key, DoubleSupplier supplier) {
            TypedInputRegistration.put(owner.doubleInputs, key, supplier);
            return this;
        }

        public InputsSection<S, SP> intVal(String key, IntSupplier supplier) {
            TypedInputRegistration.put(owner.intInputs, key, supplier);
            return this;
        }

        public InputsSection<S, SP> stringVal(String key, Supplier<String> supplier) {
            TypedInputRegistration.put(owner.stringInputs, key, supplier);
            return this;
        }

        public InputsSection<S, SP> pose2dVal(String key, Supplier<Pose2d> supplier) {
            TypedInputRegistration.put(owner.pose2dInputs, key, supplier);
            return this;
        }

        public InputsSection<S, SP> pose3dVal(String key, Supplier<Pose3d> supplier) {
            TypedInputRegistration.put(owner.pose3dInputs, key, supplier);
            return this;
        }

        public InputsSection<S, SP> objVal(String key, Supplier<?> supplier) {
            TypedInputRegistration.put(owner.objectInputs, key, supplier);
            return this;
        }
    }

    @FunctionalInterface
    private interface ChildFactory<SP> {
        SuperstructureMechanism.Child<SP, ?> build();
    }

    @FunctionalInterface
    public interface Binding<SP> {
        void apply(SuperstructureContext<SP> context);
    }

    @FunctionalInterface
    public interface TransitionHook<SP, S extends Enum<S>> {
        void apply(SuperstructureContext<SP> context, S from, S to);
    }

    public record StateTransitionPair<S extends Enum<S>>(S from, S to) { }

    public record TransitionBinding<SP, S extends Enum<S>>(S from, S to, TransitionHook<SP, S> hook) {
        public TransitionBinding {
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            Objects.requireNonNull(hook, "hook");
        }
    }

    public record LifecycleHookBinding<SP, S extends Enum<S>>(
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


    static final class Attachment<SP, E extends Enum<E> & SetpointProvider<Double>> {
        final Function<SP, E> childMapper;
        final Function<SuperstructureContext<SP>, Mechanism> resolver;
        final Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier;

        private Attachment(Function<SP, E> childMapper,
                            Function<SuperstructureContext<SP>, Mechanism> resolver,
                            Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier) {
            this.childMapper = childMapper;
            this.resolver = resolver;
            this.poseSupplier = poseSupplier;
        }

        static <SP, E extends Enum<E> & SetpointProvider<Double>> Attachment<SP, E> forMapper(
                Function<SP, E> childMapper,
                Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier) {
            return new Attachment<>(childMapper, null, poseSupplier);
        }

        static <SP, E extends Enum<E> & SetpointProvider<Double>> Attachment<SP, E> forResolver(
                Function<SuperstructureContext<SP>, Mechanism> resolver,
                Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier) {
            return new Attachment<>(null, resolver, poseSupplier);
        }
    }

    public static final class Constraint<S, SP> {
        final Predicate<SuperstructureContext<SP>> guard;
        final List<S> transitionStates;

        Constraint(Predicate<SuperstructureContext<SP>> guard, List<S> transitionStates) {
            this.guard = Objects.requireNonNull(guard, "guard");
            this.transitionStates = List.copyOf(Objects.requireNonNull(transitionStates, "transitionStates"));
        }
    }

    private Map<S, Constraint<S, SP>> mergedConstraints() {
        return new HashMap<>(constraints);
    }

    private Map<S, List<Binding<SP>>> mergedBindings() {
        Map<S, List<Binding<SP>>> merged = new HashMap<>();
        for (Map.Entry<S, List<Binding<SP>>> entry : bindings.entrySet()) {
            merged.put(entry.getKey(), List.copyOf(entry.getValue()));
        }
        return merged;
    }

    private Map<S, List<Binding<SP>>> mergedEnterBindings() {
        Map<S, List<Binding<SP>>> merged = new HashMap<>();
        for (Map.Entry<S, List<Binding<SP>>> entry : enterBindings.entrySet()) {
            merged.put(entry.getKey(), List.copyOf(entry.getValue()));
        }
        return merged;
    }

    private List<Binding<SP>> mergedAlwaysBindings() {
        return List.copyOf(alwaysBindings);
    }

    private List<Binding<SP>> mergedPeriodicBindings() {
        return List.copyOf(periodicBindings);
    }

    private Map<S, List<Binding<SP>>> mergedExitBindings() {
        Map<S, List<Binding<SP>>> merged = new HashMap<>();
        for (Map.Entry<S, List<Binding<SP>>> entry : exitBindings.entrySet()) {
            merged.put(entry.getKey(), List.copyOf(entry.getValue()));
        }
        return merged;
    }

    private List<Binding<SP>> mergedExitAlwaysBindings() {
        return List.copyOf(exitAlwaysBindings);
    }

    /**
     * Fluent helper used by {@link SuperstructureConfig#sim(Consumer)} to declare attachments.
     */
    public static final class SimulationBuilder<SP> {
        private final List<Attachment<SP, ?>> attachments = new ArrayList<>();

        /**
         * Anchors a child mechanism's visualization root to a supplied pose.
         *
         * @param childMapper mapper that identifies the child mechanism
         * @param poseSupplier computes the desired pose using the superstructure context
         */
        public <E extends Enum<E> & SetpointProvider<Double>> SimulationBuilder<SP> attach(
                Function<SP, E> childMapper,
                Function<SuperstructureContext<SP>, Pose3d> poseSupplier) {
            Objects.requireNonNull(childMapper, "childMapper");
            Objects.requireNonNull(poseSupplier, "poseSupplier");
            attachments.add(Attachment.forMapper(childMapper, poseSupplier));
            return this;
        }

        /**
         * Anchors a child mechanism's visualization root using a resolver that can reach nested mechanisms.
         *
         * @param resolver resolves the mechanism to anchor
         * @param poseSupplier computes the desired pose using the superstructure context
         */
        public SimulationBuilder<SP> attachResolved(
                Function<SuperstructureContext<SP>, Mechanism> resolver,
                Function<SuperstructureContext<SP>, Pose3d> poseSupplier) {
            Objects.requireNonNull(resolver, "resolver");
            Objects.requireNonNull(poseSupplier, "poseSupplier");
            attachments.add(Attachment.forResolver(resolver, poseSupplier));
            return this;
        }
    }
}
