package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Pose3d;
import ca.frc6390.athena.mechanisms.StatefulLike;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;

/**
 * Declarative config for a superstructure that coordinates multiple stateful mechanisms.
 * Callers map each superstate setpoint to the child mechanism state via a mapper function
 * (for example, {@code addMechanism(ELEVATOR_CONFIG, SuperTuple::elev)}). Guards can reference
 * the child mechanisms through {@link SuperstructureContext#getMechanisms()} without
 * naming them explicitly.
 *
 * @param <S> superstate enum type
 * @param <SP> setpoint tuple returned by the superstate enum
 */
public final class SuperstructureConfig<S extends Enum<S> & SetpointProvider<SP>, SP> {

    public S initialState;
    public double stateMachineDelaySeconds = 0.0;
    public List<ChildFactory<SP>> childConfigs = new ArrayList<>();
    public Map<S, Predicate<SuperstructureContext<SP>>> guards = new HashMap<>();
    public List<Attachment<SP, ?>> attachments = new ArrayList<>();
    public Map<String, BooleanSupplier> inputs = new HashMap<>();
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
        return new SuperstructureMechanism<>(initialState, stateMachineDelaySeconds, builtChildren, guards, attachments, inputs);
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
        return new SuperstructureMechanism<>(initialState, stateMachineDelaySeconds, builtChildren, guards, attachments, merged);
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
     * Adds a guard for a specific superstate. The guard can inspect child mechanisms through
     * {@link SuperstructureContext#getMechanisms()} using the same mapper passed to
     * {@link #addMech(MechanismConfig, Function)}.
     *
     * @param state superstate to guard
     * @param guard predicate that must be true for the transition to occur
     * @return this config for chaining
     */
    public SuperstructureConfig<S, SP> addGuard(S state, Predicate<SuperstructureContext<SP>> guard) {
        guards.put(Objects.requireNonNull(state, "state"), Objects.requireNonNull(guard, "guard"));
        return this;
    }

    /**
     * Groups simulation/visualization attachments in a single call to mirror the mechanism
     * style {@code .setSimulation(...)} usage.
     *
     * Example:
     * <pre>
     *   .setSimulation(sim -> sim.attach(SuperTuple::arm,
     *       ctx -> new Pose3d(0, 0, ctx.mechanism(SuperTuple::elev).getPositionMeters(), new Rotation3d())))
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
     * Adds a named external input (BooleanSupplier) that can be read in guards via {@link SuperstructureContext#input(String)}.
     */
    public SuperstructureConfig<S, SP> addInput(String key, BooleanSupplier supplier) {
        inputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
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
        attachments.add(new Attachment<>(childMapper, poseSupplier));
        return this;
    }

    @FunctionalInterface
    private interface ChildFactory<SP> {
        SuperstructureMechanism.Child<SP, ?> build();
    }

    static final class Attachment<SP, E extends Enum<E> & SetpointProvider<Double>> {
        final Function<SP, E> childMapper;
        final Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier;

        Attachment(Function<SP, E> childMapper,
                   Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier) {
            this.childMapper = childMapper;
            this.poseSupplier = poseSupplier;
        }
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
            attachments.add(new Attachment<>(childMapper, poseSupplier));
            return this;
        }
    }
}
