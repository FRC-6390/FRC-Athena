package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Pose3d;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import ca.frc6390.athena.mechanisms.Mechanism;

/**
 * Declarative config for a superstructure that coordinates multiple stateful mechanisms.
 * Callers map each superstate setpoint to the child mechanism state via a mapper function
 * (for example, {@code addMechanism(ELEVATOR_CONFIG, SuperTuple::elev)}). Constraints can
 * reference the child mechanisms through {@link SuperstructureContext#getMechanisms()} without
 * naming them explicitly, and can optionally insert transition states before a guarded state.
 *
 * @param <S> superstate enum type
 * @param <SP> setpoint tuple returned by the superstate enum
 */
public final class SuperstructureConfig<S extends Enum<S> & SetpointProvider<SP>, SP> {

    public S initialState;
    public double stateMachineDelaySeconds = 0.0;
    public List<ChildFactory<SP>> childConfigs = new ArrayList<>();
    public Map<S, Constraint<S, SP>> constraints = new HashMap<>();
    public List<Attachment<SP, ?>> attachments = new ArrayList<>();
    public Map<String, BooleanSupplier> inputs = new HashMap<>();
    public Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
    public Map<String, Supplier<?>> objectInputs = new HashMap<>();
    public Map<S, List<Binding<SP>>> bindings = new HashMap<>();
    public List<Binding<SP>> alwaysBindings = new ArrayList<>();
    private final SP initialSetpoint;

    public SuperstructureConfig(S initialState) {
        this.initialState = Objects.requireNonNull(initialState, "initialState");
        this.initialSetpoint = initialState.getSetpoint();
    }

    public static <S extends Enum<S> & SetpointProvider<SP>, SP> SuperstructureConfig<S, SP> create(S initialState) {
        return new SuperstructureConfig<>(initialState);
    }

    public SuperstructureMechanism<S, SP> build() {
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        return new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                inputs,
                doubleInputs,
                objectInputs,
                mergedBindings(),
                mergedAlwaysBindings());
    }

    /**
     * Builds a superstructure with additional/overridden inputs supplied at build time.
     */
    public SuperstructureMechanism<S, SP> build(Map<String, BooleanSupplier> inputOverrides) {
        Map<String, BooleanSupplier> merged = new HashMap<>(inputs);
        merged.putAll(inputOverrides);
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        return new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                merged,
                doubleInputs,
                objectInputs,
                mergedBindings(),
                mergedAlwaysBindings());
    }

    /**
     * Builds a superstructure with additional/overridden inputs supplied at build time.
     */
    public SuperstructureMechanism<S, SP> build(Map<String, BooleanSupplier> booleanOverrides,
                                               Map<String, DoubleSupplier> doubleOverrides,
                                               Map<String, Supplier<?>> objectOverrides) {
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
        return new SuperstructureMechanism<>(
                initialState,
                stateMachineDelaySeconds,
                builtChildren,
                mergedConstraints(),
                attachments,
                mergedBooleans,
                mergedDoubles,
                mergedObjects,
                mergedBindings(),
                mergedAlwaysBindings());
    }

    /**
     * Sets the state machine delay (seconds).
     */
    public SuperstructureConfig<S, SP> setStateMachineDelay(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
        return this;
    }

    /**
     * Adds a child mechanism and the mapper that extracts its desired state from the supertuple.
     *
     * @param config mechanism configuration (will be built when the superstructure is built)
     * @param mapper maps the supertuple to the child mechanism's state enum
     * @param <E> child mechanism state enum type
     * @return this config for chaining
     */
    public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> SuperstructureConfig<S, SP> addMech(
            MechanismConfig<T> config,
            Function<SP, E> mapper) {

        Objects.requireNonNull(config, "config");
        Objects.requireNonNull(mapper, "mapper");

        E sample = mapper.apply(initialSetpoint);
        Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
        childConfigs.add(() -> {
            Mechanism mechanism = config.build();
            return new SuperstructureMechanism.Child<>(mechanism, mapper, stateType);
        });
        return this;
    }

    /**
     * Adds a child mechanism and the mapper that extracts its desired state from the supertuple.
     *
     * @param config mechanism configuration (will be built when the superstructure is built)
     * @param mapper maps the supertuple to the child mechanism's state enum
     * @param <E> child mechanism state enum type
     * @return this config for chaining
     */
    public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> SuperstructureConfig<S, SP> addMechanism(
            MechanismConfig<T> config,
            Function<SP, E> mapper) {
        return addMech(config, mapper);
    }

    /**
     * Adds an already-constructed stateful mechanism (useful when the caller needs to retain
     * direct references for registration or visualization).
     *
     * @param mechanism existing mechanism instance
     * @param mapper maps the supertuple to the child mechanism's state enum
     * @param <E> child mechanism state enum type
     * @return this config for chaining
     */
    public <E extends Enum<E> & SetpointProvider<Double>> SuperstructureConfig<S, SP> addExistingMech(
            StatefulMechanism<E> mechanism,
            Function<SP, E> mapper) {

        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(mapper, "mapper");

        E sample = mapper.apply(initialSetpoint);
        Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
        childConfigs.add(() -> new SuperstructureMechanism.Child<>(mechanism, mapper, stateType));
        return this;
    }

    /**
     * Adds an already-constructed stateful mechanism that implements {@link StatefulLike}.
     *
     * @param mechanism existing mechanism instance
     * @param mapper maps the supertuple to the child mechanism's state enum
     * @param <E> child mechanism state enum type
     * @param <T> mechanism type
     * @return this config for chaining
     */
    public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> SuperstructureConfig<S, SP> addExistingMech(
            T mechanism,
            Function<SP, E> mapper) {

        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(mapper, "mapper");

        E sample = mapper.apply(initialSetpoint);
        Class<E> stateType = sample != null ? sample.getDeclaringClass() : null;
        childConfigs.add(() -> new SuperstructureMechanism.Child<>(mechanism, mapper, stateType));
        return this;
    }

    /**
     * Adds an already-constructed stateful mechanism using the addMechanism syntax.
     *
     * @param mechanism existing mechanism instance
     * @param mapper maps the supertuple to the child mechanism's state enum
     * @param <E> child mechanism state enum type
     * @return this config for chaining
     */
    public <E extends Enum<E> & SetpointProvider<Double>> SuperstructureConfig<S, SP> addExistingMechanism(
            StatefulMechanism<E> mechanism,
            Function<SP, E> mapper) {
        return addExistingMech(mechanism, mapper);
    }

    /**
     * Adds an already-constructed stateful mechanism using the addMechanism syntax.
     */
    public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> SuperstructureConfig<S, SP> addExistingMechanism(
            T mechanism,
            Function<SP, E> mapper) {
        return addExistingMech(mechanism, mapper);
    }

    /**
     * Adds a nested superstructure, enabling recursive composition.
     *
     * @param config nested superstructure configuration
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addSuperstructure(
            SuperstructureConfig<CS, CSP> config,
            Function<SP, CS> mapper) {

        Objects.requireNonNull(config, "config");
        Objects.requireNonNull(mapper, "mapper");

        CS sample = mapper.apply(initialSetpoint);
        Class<CS> stateType = sample != null ? sample.getDeclaringClass() : null;
        childConfigs.add(() -> {
            SuperstructureMechanism<CS, CSP> superstructure = config.build();
            return new SuperstructureMechanism.Child<>(superstructure, mapper, stateType);
        });
        return this;
    }

    /**
     * Adds a nested superstructure using the addMechanism syntax.
     *
     * @param config nested superstructure configuration
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addMechanism(
            SuperstructureConfig<CS, CSP> config,
            Function<SP, CS> mapper) {
        return addSuperstructure(config, mapper);
    }

    /**
     * Adds a nested superstructure using the same addMech syntax as leaf mechanisms.
     *
     * @param config nested superstructure configuration
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addMech(
            SuperstructureConfig<CS, CSP> config,
            Function<SP, CS> mapper) {
        return addSuperstructure(config, mapper);
    }

    /**
     * Adds an already-constructed nested superstructure.
     *
     * @param superstructure existing superstructure instance
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addExistingSuperstructure(
            SuperstructureMechanism<CS, CSP> superstructure,
            Function<SP, CS> mapper) {

        Objects.requireNonNull(superstructure, "superstructure");
        Objects.requireNonNull(mapper, "mapper");

        CS sample = mapper.apply(initialSetpoint);
        Class<CS> stateType = sample != null ? sample.getDeclaringClass() : null;
        childConfigs.add(() -> new SuperstructureMechanism.Child<>(superstructure, mapper, stateType));
        return this;
    }

    /**
     * Adds an already-constructed nested superstructure using the addMechanism syntax.
     *
     * @param superstructure existing superstructure instance
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addMechanism(
            SuperstructureMechanism<CS, CSP> superstructure,
            Function<SP, CS> mapper) {
        return addExistingSuperstructure(superstructure, mapper);
    }

    /**
     * Adds an already-constructed nested superstructure using the addMech syntax.
     *
     * @param superstructure existing superstructure instance
     * @param mapper maps the parent supertuple to the nested superstructure's state enum
     * @param <CS> nested superstructure state enum type
     * @param <CSP> nested superstructure setpoint tuple type
     * @return this config for chaining
     */
    public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureConfig<S, SP> addMech(
            SuperstructureMechanism<CS, CSP> superstructure,
            Function<SP, CS> mapper) {
        return addExistingSuperstructure(superstructure, mapper);
    }

    /**
     * Adds a constraint for a specific superstate. The guard can inspect child mechanisms through
     * {@link SuperstructureContext#getMechanisms()} using the same mapper passed to
     * {@link #addMech(MechanismConfig, Function)}. If transition states are provided and the guard
     * is not satisfied when the state is queued, the transition states are queued first.
     *
     * @param state superstate to constrain
     * @param guard predicate that must be true for the transition to occur
     * @return this config for chaining
     */
    @SafeVarargs
    public final SuperstructureConfig<S, SP> addConstraint(
            S state,
            Predicate<SuperstructureContext<SP>> guard,
            S... transitionStates) {
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(guard, "guard");
        List<S> transitions = transitionStates == null ? List.of() : List.of(transitionStates);
        transitions.forEach(t -> Objects.requireNonNull(t, "transitionStates cannot contain null"));
        Constraint<S, SP> constraint = new Constraint<>(guard, transitions);
        constraints.put(state, constraint);
        return this;
    }

    /**
     * Groups simulation/visualization attachments in a single call to mirror the mechanism
     * style {@code .setSimulation(...)} usage.
     *
     * Example:
     * <pre>
     *   .setSimulation(sim -> sim.attach(SuperTuple::arm,
     *       ctx -> new Pose3d(0, 0, ctx.getMechanisms().generic(SuperTuple::elev).getPositionMeters(), new Rotation3d())))
     * </pre>
     *
     * @param configurer callback that adds attachments
     */
    public SuperstructureConfig<S, SP> setSimulation(Consumer<SimulationBuilder<SP>> configurer) {
        Objects.requireNonNull(configurer, "configurer");
        SimulationBuilder<SP> simBuilder = new SimulationBuilder<>();
        configurer.accept(simBuilder);
        attachments.addAll(simBuilder.attachments);
        return this;
    }

    /**
     * Adds a named external input (BooleanSupplier) that can be read in constraints via {@link SuperstructureContext#input(String)}.
     */
    public SuperstructureConfig<S, SP> addInput(String key, BooleanSupplier supplier) {
        inputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Adds a named boolean input (alias for {@link #addInput(String, BooleanSupplier)}).
     */
    public SuperstructureConfig<S, SP> addBooleanInput(String key, BooleanSupplier supplier) {
        return addInput(key, supplier);
    }

    /**
     * Adds a named double input that can be read via {@link SuperstructureContext#doubleInput(String)}.
     */
    public SuperstructureConfig<S, SP> addDoubleInput(String key, DoubleSupplier supplier) {
        doubleInputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Adds a named object input that can be read via {@link SuperstructureContext#objectInput(String, Class)}.
     */
    public SuperstructureConfig<S, SP> addObjectInput(String key, Supplier<?> supplier) {
        objectInputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Adds a named input sourced from a limit switch configuration. Any provided key will be
     * used even if the config has its own name.
     */
    public SuperstructureConfig<S, SP> addInput(String key, GenericLimitSwitchConfig config) {
        Objects.requireNonNull(config, "config");
        return addInput(key, config.setName(key).toSupplier());
    }

    /**
     * Adds a limit switch as an input. If the config has a name, it will be used as the key unless
     * a key is provided.
     */
    public SuperstructureConfig<S, SP> addInput(GenericLimitSwitchConfig config) {
        String key = config.name() != null ? config.name() : ("dio-" + config.id());
        inputs.put(key, config.toSupplier());
        return this;
    }

    /**
     * Overrides the visualization root pose for a child mechanism so it can be anchored to
     * another mechanism (e.g., arm attached to elevator carriage). If the supplier returns
     * {@code null}, the child falls back to its own root pose supplier.
     *
     * @param childMapper mapper that identifies the child mechanism to anchor
     * @param poseSupplier computes the desired root pose using the superstructure context
     */
    public <E extends Enum<E> & SetpointProvider<Double>> SuperstructureConfig<S, SP> addAttachment(
            Function<SP, E> childMapper,
            Function<SuperstructureContext<SP>, Pose3d> poseSupplier) {
        Objects.requireNonNull(childMapper, "childMapper");
        Objects.requireNonNull(poseSupplier, "poseSupplier");
        attachments.add(Attachment.forMapper(childMapper, poseSupplier));
        return this;
    }

    /**
     * Registers a hook that runs every loop while the specified superstates are active.
     */
    @SafeVarargs
    public final SuperstructureConfig<S, SP> addOnStateHook(Binding<SP> binding, S... states) {
        Objects.requireNonNull(binding, "binding");
        if (states == null || states.length == 0) {
            alwaysBindings.add(binding);
            return this;
        }
        for (S state : states) {
            Objects.requireNonNull(state, "states cannot contain null");
            bindings.computeIfAbsent(state, key -> new ArrayList<>()).add(binding);
        }
        return this;
    }

    /**
     * Registers a hook that runs every loop regardless of the active superstate.
     */
    public SuperstructureConfig<S, SP> addOnStateHook(Binding<SP> binding) {
        Objects.requireNonNull(binding, "binding");
        alwaysBindings.add(binding);
        return this;
    }


    @FunctionalInterface
    private interface ChildFactory<SP> {
        SuperstructureMechanism.Child<SP, ?> build();
    }

    @FunctionalInterface
    public interface Binding<SP> {
        void apply(SuperstructureContext<SP> context);
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

    private List<Binding<SP>> mergedAlwaysBindings() {
        return List.copyOf(alwaysBindings);
    }

    /**
     * Fluent helper used by {@link SuperstructureConfig#setSimulation(Consumer)} to declare attachments.
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
