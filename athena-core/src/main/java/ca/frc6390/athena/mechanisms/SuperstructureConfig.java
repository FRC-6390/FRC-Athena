package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.geometry.Pose3d;
import ca.frc6390.athena.mechanisms.StatefulLike;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;

/**
 * Declarative builder for a superstructure that coordinates multiple stateful mechanisms.
 * Callers map each superstate setpoint to the child mechanism state via a mapper function
 * (for example, {@code addMech(ELEVATOR_CONFIG, SuperTuple::elev)}). Guards can reference
 * the child mechanisms through {@link SuperstructureContext#mechanism(Function)} without
 * naming them explicitly.
 *
 * @param <S> superstate enum type
 * @param <SP> setpoint tuple returned by the superstate enum
 */
public final class SuperstructureConfig<S extends Enum<S> & SetpointProvider<SP>, SP> {

    private final S initialState;
    private final double stateMachineDelaySeconds;
    private final List<ChildFactory<SP>> childConfigs;
    private final Map<S, Predicate<SuperstructureContext<SP>>> guards;
    private final List<Attachment<SP, ?>> attachments;
    private final Map<String, java.util.function.BooleanSupplier> inputs;

    private SuperstructureConfig(S initialState,
                                 double stateMachineDelaySeconds,
                                 List<ChildFactory<SP>> childConfigs,
                                 Map<S, Predicate<SuperstructureContext<SP>>> guards,
                                 List<Attachment<SP, ?>> attachments,
                                 Map<String, java.util.function.BooleanSupplier> inputs) {
        this.initialState = initialState;
        this.stateMachineDelaySeconds = stateMachineDelaySeconds;
        this.childConfigs = childConfigs;
        this.guards = guards;
        this.attachments = attachments;
        this.inputs = inputs;
    }

    public static <S extends Enum<S> & SetpointProvider<SP>, SP> Builder<S, SP> builder(S initialState) {
        return new Builder<>(initialState);
    }

    public static <S extends Enum<S> & SetpointProvider<SP>, SP> Builder<S, SP> create(S initialState) {
        return builder(initialState);
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
    public SuperstructureMechanism<S, SP> build(Map<String, java.util.function.BooleanSupplier> inputOverrides) {
        Map<String, java.util.function.BooleanSupplier> merged = new HashMap<>(inputs);
        merged.putAll(inputOverrides);
        List<SuperstructureMechanism.Child<SP, ?>> builtChildren = new ArrayList<>();
        for (ChildFactory<SP> child : childConfigs) {
            builtChildren.add(child.build());
        }
        return new SuperstructureMechanism<>(initialState, stateMachineDelaySeconds, builtChildren, guards, attachments, merged);
    }

    /**
     * Builder that captures the child mechanisms and guards before constructing the composite.
     */
    public static final class Builder<S extends Enum<S> & SetpointProvider<SP>, SP> {

        private final S initialState;
        private final SP initialSetpoint;
        private double stateMachineDelaySeconds = 0.0;
        private final List<ChildFactory<SP>> childConfigs = new ArrayList<>();
        private final Map<S, Predicate<SuperstructureContext<SP>>> guards = new HashMap<>();
        private final List<Attachment<SP, ?>> attachments = new ArrayList<>();
        private final Map<String, java.util.function.BooleanSupplier> inputs = new HashMap<>();

        private Builder(S initialState) {
            this.initialState = Objects.requireNonNull(initialState, "initialState");
            this.initialSetpoint = initialState.getSetpoint();
        }

        public Builder<S, SP> setStateMachineDelay(double delaySeconds) {
            this.stateMachineDelaySeconds = delaySeconds;
            return this;
        }

        /**
         * Adds a child mechanism and the mapper that extracts its desired state from the supertuple.
         *
         * @param config mechanism configuration (will be built when the superstructure is built)
         * @param mapper maps the supertuple to the child mechanism's state enum
         * @param <E> child mechanism state enum type
         * @return this builder for chaining
         */
        public <E extends Enum<E> & SetpointProvider<Double>, T extends Mechanism & StatefulLike<E>> Builder<S, SP> addMech(
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
         * Adds an already-constructed stateful mechanism (useful when the caller needs to retain
         * direct references for registration or visualization).
         *
         * @param mechanism existing mechanism instance
         * @param mapper maps the supertuple to the child mechanism's state enum
         * @param <E> child mechanism state enum type
         * @return this builder for chaining
         */
        public <E extends Enum<E> & SetpointProvider<Double>> Builder<S, SP> addExistingMech(
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
         * Adds a nested superstructure, enabling recursive composition.
         *
         * @param config nested superstructure configuration
         * @param mapper maps the parent supertuple to the nested superstructure's state enum
         * @param <CS> nested superstructure state enum type
         * @param <CSP> nested superstructure setpoint tuple type
         * @return this builder for chaining
         */
        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> Builder<S, SP> addSuperstructure(
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
         * Adds an already-constructed nested superstructure.
         *
         * @param superstructure existing superstructure instance
         * @param mapper maps the parent supertuple to the nested superstructure's state enum
         * @param <CS> nested superstructure state enum type
         * @param <CSP> nested superstructure setpoint tuple type
         * @return this builder for chaining
         */
        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> Builder<S, SP> addExistingSuperstructure(
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
         * Adds a guard for a specific superstate. The guard can inspect child mechanisms through
         * {@link SuperstructureContext#mechanism(Function)} using the same mapper passed to
         * {@link #addMech(MechanismConfig, Function)}.
         *
         * @param state superstate to guard
         * @param guard predicate that must be true for the transition to occur
         * @return this builder for chaining
         */
        public Builder<S, SP> addGuard(S state, Predicate<SuperstructureContext<SP>> guard) {
            guards.put(Objects.requireNonNull(state, "state"), Objects.requireNonNull(guard, "guard"));
            return this;
        }

        public SuperstructureConfig<S, SP> buildConfig() {
            return new SuperstructureConfig<>(initialState, stateMachineDelaySeconds,
                    List.copyOf(childConfigs), Map.copyOf(guards), List.copyOf(attachments), Map.copyOf(inputs));
        }

        public SuperstructureMechanism<S, SP> build() {
            return buildConfig().build();
        }

        public SuperstructureMechanism<S, SP> build(Map<String, java.util.function.BooleanSupplier> inputOverrides) {
            return buildConfig().build(inputOverrides);
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
        public Builder<S, SP> setSimulation(Consumer<SimulationBuilder<SP>> configurer) {
            Objects.requireNonNull(configurer, "configurer");
            SimulationBuilder<SP> simBuilder = new SimulationBuilder<>();
            configurer.accept(simBuilder);
            attachments.addAll(simBuilder.attachments);
            return this;
        }

        /**
         * Adds a named external input (BooleanSupplier) that can be read in guards via {@link SuperstructureContext#input(String)}.
         */
        public Builder<S, SP> addInput(String key, java.util.function.BooleanSupplier supplier) {
            inputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
            return this;
        }

        /**
         * Adds a named input sourced from a limit switch configuration. Any provided key will be
         * used even if the config has its own name.
         */
        public Builder<S, SP> addInput(String key, GenericLimitSwitchConfig config) {
            Objects.requireNonNull(config, "config");
            return addInput(key, config.setName(key).toSupplier());
        }

        /**
         * Adds a limit switch as an input. If the config has a name, it will be used as the key unless
         * a key is provided.
         */
        public Builder<S, SP> addInput(GenericLimitSwitchConfig config) {
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
        public <E extends Enum<E> & SetpointProvider<Double>> Builder<S, SP> addAttachment(
                Function<SP, E> childMapper,
                Function<SuperstructureContext<SP>, edu.wpi.first.math.geometry.Pose3d> poseSupplier) {
            Objects.requireNonNull(childMapper, "childMapper");
            Objects.requireNonNull(poseSupplier, "poseSupplier");
            attachments.add(new Attachment<>(childMapper, poseSupplier));
            return this;
        }
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
     * Fluent helper used by {@link Builder#setSimulation(Consumer)} to declare attachments.
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
