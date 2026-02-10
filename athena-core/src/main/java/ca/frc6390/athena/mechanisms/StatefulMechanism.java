package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class StatefulMechanism <E extends Enum<E> & SetpointProvider<Double>> extends Mechanism implements StatefulLike<E> {
        
    private final StatefulMechanismCore<StatefulMechanism<E>, E> stateCore;

    public StatefulMechanism(MechanismConfig<StatefulMechanism<E>> config, E initialState) {
        super(config);
        stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.data().stateMachineDelay(),
                config.stateActions,
                config.enterStateHooks,
                config.stateHooks,
                config.exitStateHooks,
                config.transitionHooks,
                config.alwaysHooks,
                config.exitAlwaysHooks,
                config.inputs,
                config.doubleInputs,
                config.intInputs,
                config.stringInputs,
                config.pose2dInputs,
                config.pose3dInputs,
                config.objectInputs,
                config.stateTriggerBindings);
    }

    @Override
    public double getSetpoint() {
        return stateCore.getSetpoint();
    }

    @Override
    protected double getBaseSetpoint() {
        return stateCore.getBaseSetpoint();
    }

    @Override
    protected Enum<?> getActiveState() {
        return stateCore.getStateMachine().getGoalState();
    }

    /**
     * Overrides the setpoint used by the state machine with a dynamic supplier.
     */
    @Override
    public void setSetpointOverride(DoubleSupplier override) {
        stateCore.setSetpointOverride(override);
    }

    /**
     * Clears any active setpoint override.
     */
    @Override
    public void clearSetpointOverride() {
        stateCore.clearSetpointOverride();
    }

    /**
     * Suppresses motor output while the supplier evaluates to true.
     */
    @Override
    public void setOutputSuppressor(BooleanSupplier suppressor) {
        stateCore.setOutputSuppressor(suppressor);
    }

    /**
     * Clears any active output suppressor.
     */
    public void clearOutputSuppressor() {
        stateCore.clearOutputSuppressor();
    }

    @Override
    public void update() {
        setSuppressMotorOutput(stateCore.update(this));
        super.update();
    }

    @Override
    public StateMachine<Double, E> getStateMachine() {
        return stateCore.getStateMachine();
    }

    public void setStateGraph(StateGraph<E> stateGraph) {
        stateCore.setStateGraph(stateGraph);
    }

    @SuppressWarnings("unchecked")
    public StatefulMechanism<E> publishNetworkTables(String ownerHint) {
        super.publishNetworkTables(ownerHint);
        return this;
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return null;
        }
        // Publish state machine topics alongside the mechanism topics.
        getStateMachine().networkTables(node.child("Control").child("StateMachine"));
        return super.networkTables(node);
    }

    public static class StatefulMechanismCore<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        private final StateMachine<Double, E> stateMachine;
        private final Map<Enum<?>, Function<T, Boolean>> stateActions;
        private final Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> enterStateHooks;
        private final Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> stateHooks;
        private final Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> exitStateHooks;
        private final List<MechanismConfig.TransitionHookBinding<T>> transitionHooks;
        private final List<MechanismConfig.MechanismBinding<T, ?>> alwaysHooks;
        private final List<MechanismConfig.MechanismBinding<T, ?>> exitAlwaysHooks;
        private final Map<String, BooleanSupplier> inputs;
        private final Map<String, DoubleSupplier> doubleInputs;
        private final Map<String, IntSupplier> intInputs;
        private final Map<String, Supplier<String>> stringInputs;
        private final Map<String, Supplier<Pose2d>> pose2dInputs;
        private final Map<String, Supplier<Pose3d>> pose3dInputs;
        private final Map<String, Supplier<?>> objectInputs;
        private final List<StateTriggerRunner<T, E>> stateTriggers;
        private final MechanismContextImpl context = new MechanismContextImpl();
        private DoubleSupplier setpointOverride;
        private BooleanSupplier outputSuppressor;
        private E previousState;

        public StatefulMechanismCore(E initialState, Supplier<Boolean> atSetpointSupplier, double delay,
                                        Map<Enum<?>, Function<T, Boolean>> stateActions,
                                        Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> enterStateHooks,
                                        Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> stateHooks,
                                        Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> exitStateHooks,
                                        List<MechanismConfig.TransitionHookBinding<T>> transitionHooks,
                                        List<MechanismConfig.MechanismBinding<T, ?>> alwaysHooks,
                                        List<MechanismConfig.MechanismBinding<T, ?>> exitAlwaysHooks,
                                        Map<String, BooleanSupplier> inputs,
                                        Map<String, DoubleSupplier> doubleInputs,
                                        Map<String, IntSupplier> intInputs,
                                        Map<String, Supplier<String>> stringInputs,
                                        Map<String, Supplier<Pose2d>> pose2dInputs,
                                        Map<String, Supplier<Pose3d>> pose3dInputs,
                                        Map<String, Supplier<?>> objectInputs,
                                        List<MechanismConfig.StateTriggerBinding<T>> stateTriggerBindings) {
            this.stateMachine = new StateMachine<>(initialState, atSetpointSupplier::get);
            stateMachine.setAtStateDelay(delay);
            this.stateActions = stateActions;
            this.enterStateHooks = enterStateHooks;
            this.stateHooks = stateHooks;
            this.exitStateHooks = exitStateHooks;
            this.transitionHooks = transitionHooks;
            this.alwaysHooks = alwaysHooks;
            this.exitAlwaysHooks = exitAlwaysHooks;
            this.inputs = inputs;
            this.doubleInputs = doubleInputs;
            this.intInputs = intInputs;
            this.stringInputs = stringInputs;
            this.pose2dInputs = pose2dInputs;
            this.pose3dInputs = pose3dInputs;
            this.objectInputs = objectInputs;
            this.previousState = initialState;
            this.stateTriggers = buildStateTriggers(stateTriggerBindings);
        }

        @SuppressWarnings("unchecked")
        private List<StateTriggerRunner<T, E>> buildStateTriggers(List<MechanismConfig.StateTriggerBinding<T>> bindings) {
            if (bindings == null || bindings.isEmpty()) {
                return List.of();
            }
            List<StateTriggerRunner<T, E>> runners = new java.util.ArrayList<>();
            for (MechanismConfig.StateTriggerBinding<T> binding : bindings) {
                if (binding == null || binding.state() == null || binding.trigger() == null) {
                    continue;
                }
                E state;
                try {
                    state = (E) binding.state();
                } catch (ClassCastException e) {
                    continue;
                }
                MechanismConfig.StateTrigger<T, E> trigger = (MechanismConfig.StateTrigger<T, E>) binding.trigger();
                runners.add(new StateTriggerRunner<>(state, trigger));
            }
            return runners;
        }

        private void evaluateStateTriggers() {
            if (stateTriggers == null || stateTriggers.isEmpty()) {
                return;
            }
            for (StateTriggerRunner<T, E> trigger : stateTriggers) {
                if (trigger == null || trigger.target == null || trigger.trigger == null) {
                    continue;
                }
                boolean now = trigger.trigger.shouldQueue(context);
                boolean rising = now && !trigger.last;
                trigger.last = now;
                if (!rising) {
                    continue;
                }
                if (stateMachine.isGoalState(trigger.target) || stateMachine.isQueued(trigger.target)) {
                    continue;
                }
                stateMachine.queueState(trigger.target);
            }
        }

        public double getSetpoint() {
            DoubleSupplier override = setpointOverride;
            if (override != null) {
                return override.getAsDouble();
            }
            return stateMachine.getGoalState().getSetpoint();
        }

        public double getBaseSetpoint() {
            return stateMachine.getGoalState().getSetpoint();
        }

        public void setSetpointOverride(DoubleSupplier override) {
            this.setpointOverride = override;
        }

        public void clearSetpointOverride() {
            this.setpointOverride = null;
        }

        public void setOutputSuppressor(BooleanSupplier suppressor) {
            this.outputSuppressor = suppressor;
        }

        public void clearOutputSuppressor() {
            this.outputSuppressor = null;
        }

        public boolean update(T instance) {
            // Update the context with the current goal state so triggers can examine live mechanism data.
            context.update(instance, stateMachine.getGoalState(), getBaseSetpoint());
            evaluateStateTriggers();
            stateMachine.update();
            E currentState = stateMachine.getGoalState();
            boolean changed = !Objects.equals(currentState, previousState);
            E from = previousState;
            if (changed) {
                applyExitHooks(instance, from);
            }
            context.update(instance, currentState, getBaseSetpoint());
            if (changed) {
                applyTransitionHooks(instance, from, currentState);
                applyEnterHooks(currentState);
                previousState = currentState;
            }
            applyHooks(currentState);
            boolean suppress = false;
            Function<T, Boolean> action = stateActions.get(currentState);
            if (action != null) {
                suppress = action.apply(instance);
            }
            if (outputSuppressor != null && outputSuppressor.getAsBoolean()) {
                suppress = true;
            }
            return suppress;
        }

        private static final class StateTriggerRunner<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
            private final E target;
            private final MechanismConfig.StateTrigger<T, E> trigger;
            private boolean last;

            private StateTriggerRunner(E target, MechanismConfig.StateTrigger<T, E> trigger) {
                this.target = target;
                this.trigger = trigger;
            }
        }

        private void applyEnterHooks(E state) {
            if (state == null) {
                return;
            }
            List<MechanismConfig.MechanismBinding<T, ?>> hooks = enterStateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (MechanismConfig.MechanismBinding<T, ?> binding : hooks) {
                applyHook(binding);
            }
        }

        @SuppressWarnings("unchecked")
        private void applyTransitionHooks(T instance, E from, E to) {
            if (from == null || to == null || transitionHooks == null || transitionHooks.isEmpty()) {
                return;
            }
            for (MechanismConfig.TransitionHookBinding<T> hook : transitionHooks) {
                if (hook == null) {
                    continue;
                }
                if (!Objects.equals(hook.from(), from) || !Objects.equals(hook.to(), to)) {
                    continue;
                }
                ((MechanismConfig.MechanismTransitionBinding<T, E>) hook.binding()).apply(context, from, to);
            }
        }

        private void applyExitHooks(T instance, E state) {
            if (state == null) {
                return;
            }
            context.update(instance, state, state.getSetpoint());
            for (MechanismConfig.MechanismBinding<T, ?> binding : exitAlwaysHooks) {
                applyHook(binding);
            }
            List<MechanismConfig.MechanismBinding<T, ?>> hooks = exitStateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (MechanismConfig.MechanismBinding<T, ?> binding : hooks) {
                applyHook(binding);
            }
        }

        private void applyHooks(E state) {
            for (MechanismConfig.MechanismBinding<T, ?> binding : alwaysHooks) {
                applyHook(binding);
            }
            List<MechanismConfig.MechanismBinding<T, ?>> hooks = stateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (MechanismConfig.MechanismBinding<T, ?> binding : hooks) {
                applyHook(binding);
            }
        }

        @SuppressWarnings("unchecked")
        private void applyHook(MechanismConfig.MechanismBinding<T, ?> binding) {
            ((MechanismConfig.MechanismBinding<T, E>) binding).apply(context);
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateMachine;
        }

        public void setStateGraph(StateGraph<E> stateGraph) {
            stateMachine.setStateGraph(stateGraph);
        }

        private final class MechanismContextImpl implements MechanismContext<T, E> {
            private T mechanism;
            private E state;
            private double baseSetpoint;

            private void update(T mechanism, E state, double baseSetpoint) {
                this.mechanism = mechanism;
                this.state = state;
                this.baseSetpoint = baseSetpoint;
            }

            @Override
            public T mechanism() {
                return mechanism;
            }

            @Override
            public E state() {
                return state;
            }

            @Override
            public double baseSetpoint() {
                return baseSetpoint;
            }

            @Override
            public boolean input(String key) {
                if (mechanism != null && mechanism.hasMutableBoolValKey(key)) {
                    return mechanism.mutableBoolVal(key);
                }
                BooleanSupplier supplier = inputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No bool input found for key " + key);
                }
                return supplier.getAsBoolean();
            }

            @Override
            public BooleanSupplier inputSupplier(String key) {
                if (mechanism != null && mechanism.hasMutableBoolValKey(key)) {
                    return () -> mechanism.mutableBoolVal(key);
                }
                BooleanSupplier supplier = inputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No input found for key " + key);
                }
                return supplier;
            }

            @Override
            public double doubleInput(String key) {
                if (mechanism != null && mechanism.hasMutableDblValKey(key)) {
                    return mechanism.mutableDblVal(key);
                }
                DoubleSupplier supplier = doubleInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No double input found for key " + key);
                }
                return supplier.getAsDouble();
            }

            @Override
            public DoubleSupplier doubleInputSupplier(String key) {
                if (mechanism != null && mechanism.hasMutableDblValKey(key)) {
                    return () -> mechanism.mutableDblVal(key);
                }
                DoubleSupplier supplier = doubleInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No double input found for key " + key);
                }
                return supplier;
            }

            @Override
            public int intVal(String key) {
                if (mechanism != null && mechanism.hasMutableIntValKey(key)) {
                    return mechanism.mutableIntVal(key);
                }
                IntSupplier supplier = intInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No int input found for key " + key);
                }
                return supplier.getAsInt();
            }

            @Override
            public IntSupplier intValSupplier(String key) {
                if (mechanism != null && mechanism.hasMutableIntValKey(key)) {
                    return () -> mechanism.mutableIntVal(key);
                }
                IntSupplier supplier = intInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No int input found for key " + key);
                }
                return supplier;
            }

            @Override
            public String stringVal(String key) {
                if (mechanism != null && mechanism.hasMutableStrValKey(key)) {
                    return mechanism.mutableStrVal(key);
                }
                Supplier<String> supplier = stringInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No string input found for key " + key);
                }
                return supplier.get();
            }

            @Override
            public Supplier<String> stringValSupplier(String key) {
                if (mechanism != null && mechanism.hasMutableStrValKey(key)) {
                    return () -> mechanism.mutableStrVal(key);
                }
                Supplier<String> supplier = stringInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No string input found for key " + key);
                }
                return supplier;
            }

            @Override
            public Pose2d pose2dVal(String key) {
                if (mechanism != null && mechanism.hasMutablePose2dValKey(key)) {
                    return mechanism.mutablePose2dVal(key);
                }
                Supplier<Pose2d> supplier = pose2dInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No Pose2d input found for key " + key);
                }
                return supplier.get();
            }

            @Override
            public Supplier<Pose2d> pose2dValSupplier(String key) {
                if (mechanism != null && mechanism.hasMutablePose2dValKey(key)) {
                    return () -> mechanism.mutablePose2dVal(key);
                }
                Supplier<Pose2d> supplier = pose2dInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No Pose2d input found for key " + key);
                }
                return supplier;
            }

            @Override
            public Pose3d pose3dVal(String key) {
                if (mechanism != null && mechanism.hasMutablePose3dValKey(key)) {
                    return mechanism.mutablePose3dVal(key);
                }
                Supplier<Pose3d> supplier = pose3dInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No Pose3d input found for key " + key);
                }
                return supplier.get();
            }

            @Override
            public Supplier<Pose3d> pose3dValSupplier(String key) {
                if (mechanism != null && mechanism.hasMutablePose3dValKey(key)) {
                    return () -> mechanism.mutablePose3dVal(key);
                }
                Supplier<Pose3d> supplier = pose3dInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No Pose3d input found for key " + key);
                }
                return supplier;
            }

            @Override
            public <V> V objectInput(String key, Class<V> type) {
                Supplier<?> supplier = objectInputs.get(key);
                if (supplier == null) {
                    return null;
                }
                Object value = supplier.get();
                if (value == null) {
                    return null;
                }
                if (!type.isInstance(value)) {
                    throw new IllegalArgumentException("Input '" + key + "' is not of type " + type.getSimpleName());
                }
                return type.cast(value);
            }

            @Override
            public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
                Supplier<?> supplier = objectInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No object input found for key " + key);
                }
                return () -> objectInput(key, type);
            }
        }
    }
}
