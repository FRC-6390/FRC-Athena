package ca.frc6390.athena.mechanisms;

import java.util.ArrayDeque;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.mechanisms.statespec.StateBuilder;
import ca.frc6390.athena.mechanisms.statespec.StateCtx;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.arcp.ARCP;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class StatefulMechanism <E> extends Mechanism implements StatefulLike<E> {
        
    private final StatefulMechanismCore<StatefulMechanism<E>, E> stateMachineCore;

    public StatefulMechanism(
            MechanismRuntimeConfig<? extends StatefulMechanism<E>> runtimeConfig,
            E initialState,
            StatefulMechanismRuntimeConfig<StatefulMechanism<E>, E> stateRuntimeConfig) {
        this(runtimeConfig, initialState, stateRuntimeConfig, MechanismLifecycleHooks.NONE);
    }

    protected StatefulMechanism(
            MechanismRuntimeConfig<? extends StatefulMechanism<E>> runtimeConfig,
            E initialState,
            StatefulMechanismRuntimeConfig<StatefulMechanism<E>, E> stateRuntimeConfig,
            MechanismLifecycleHooks lifecycleHooks) {
        super(
                runtimeConfig,
                runtimeConfig.sourceKey() != null ? runtimeConfig.sourceKey() : runtimeConfig,
                lifecycleHooks);
        stateMachineCore = StatefulMechanismCore.fromRuntimeConfig(
                initialState,
                this::atSetpoint,
                stateRuntimeConfig);
        Double initialSetpoint = StateSpecAccess.setpoint(initialState);
        if (initialSetpoint != null) {
            control().setpoint(initialSetpoint);
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    public static <E> StatefulMechanism<E> create(
            MechanismRuntimeConfig<?> runtimeConfig,
            E initialState,
            StatefulMechanismRuntimeConfig<?, E> stateRuntimeConfig) {
        return new StatefulMechanism((MechanismRuntimeConfig) runtimeConfig, initialState, (StatefulMechanismRuntimeConfig) stateRuntimeConfig);
    }

    @Override
    public StatefulLike.StateMachineSection<E> stateMachine() {
        return new StatefulLike.StateMachineSection<>(stateMachineCore);
    }

    @Override
    protected Object getActiveState() {
        return stateMachineCore.getStateMachine().getGoalState();
    }

    @Override
    public void update() {
        stateMachineCore.updateMechanism(this);
        super.update();
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return null;
        }
        // Publish state machine topics alongside the mechanism topics.
        stateMachineCore.getStateMachine().networkTables(node.child("Control").child("StateMachine"));
        return super.networkTables(node);
    }

    @Override
    public void publishArcp(ARCP publisher, String rootPath) {
        if (publisher == null || rootPath == null || rootPath.isBlank()) {
            return;
        }
        stateMachineCore.getStateMachine().publishArcp(publisher, rootPath + "/Control/StateMachine");
        super.publishArcp(publisher, rootPath);
    }

    public static class StatefulMechanismCore<T extends Mechanism, E> {
        private final StateMachine<Double, E> stateMachine;
        private final Map<Object, Function<T, Boolean>> stateActions;
        private final Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> enterStateHooks;
        private final Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> stateHooks;
        private final Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> exitStateHooks;
        private final List<StatefulMechanismRuntimeConfig.TransitionHookBinding<T, E>> transitionHooks;
        private final List<StatefulMechanismRuntimeConfig.StateHook<T, E>> alwaysHooks;
        private final List<StatefulMechanismRuntimeConfig.StateHook<T, E>> exitAlwaysHooks;
        private final Map<String, BooleanSupplier> inputs;
        private final Map<String, DoubleSupplier> doubleInputs;
        private final Map<String, IntSupplier> intInputs;
        private final Map<String, Supplier<String>> stringInputs;
        private final Map<String, Supplier<Pose2d>> pose2dInputs;
        private final Map<String, Supplier<Pose3d>> pose3dInputs;
        private final Map<String, Supplier<?>> objectInputs;
        private final List<StateTriggerRunner<T, E>> stateTriggers;
        private final MechanismContextImpl context = new MechanismContextImpl();
        private final Queue<Double> queuedSetpoints = new ArrayDeque<>();
        private Double queuedSetpoint;
        private double lastFiniteSetpoint;
        private E previousState;
        private boolean dslManualOverrideActive;

        public static <T extends Mechanism, E> StatefulMechanismCore<T, E> fromRuntimeConfig(
                E initialState,
                Supplier<Boolean> atSetpointSupplier,
                StatefulMechanismRuntimeConfig<T, E> config) {
            Objects.requireNonNull(config, "config");
            return new StatefulMechanismCore<>(
                    initialState,
                    atSetpointSupplier,
                    config.delay(),
                    config.stateActions(),
                    config.enterStateHooks(),
                    config.stateHooks(),
                    config.exitStateHooks(),
                    config.transitionHooks(),
                    config.alwaysHooks(),
                    config.exitAlwaysHooks(),
                    config.inputs(),
                    config.doubleInputs(),
                    config.intInputs(),
                    config.stringInputs(),
                    config.pose2dInputs(),
                    config.pose3dInputs(),
                    config.objectInputs(),
                    config.stateTriggers());
        }

        public StatefulMechanismCore(E initialState, Supplier<Boolean> atSetpointSupplier, double delay,
                                        Map<Object, Function<T, Boolean>> stateActions,
                                        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> enterStateHooks,
                                        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> stateHooks,
                                        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> exitStateHooks,
                                        List<StatefulMechanismRuntimeConfig.TransitionHookBinding<T, E>> transitionHooks,
                                        List<StatefulMechanismRuntimeConfig.StateHook<T, E>> alwaysHooks,
                                        List<StatefulMechanismRuntimeConfig.StateHook<T, E>> exitAlwaysHooks,
                                        Map<String, BooleanSupplier> inputs,
                                        Map<String, DoubleSupplier> doubleInputs,
                                        Map<String, IntSupplier> intInputs,
                                        Map<String, Supplier<String>> stringInputs,
                                        Map<String, Supplier<Pose2d>> pose2dInputs,
                                        Map<String, Supplier<Pose3d>> pose3dInputs,
                                        Map<String, Supplier<?>> objectInputs,
                                        List<StatefulMechanismRuntimeConfig.StateTriggerBinding<T, E>> stateTriggerBindings) {
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
            Double initialSetpoint = StateSpecAccess.setpoint(initialState);
            this.lastFiniteSetpoint = initialSetpoint != null && Double.isFinite(initialSetpoint)
                    ? initialSetpoint
                    : 0.0;
        }

        private List<StateTriggerRunner<T, E>> buildStateTriggers(
                List<StatefulMechanismRuntimeConfig.StateTriggerBinding<T, E>> bindings) {
            if (bindings == null || bindings.isEmpty()) {
                return List.of();
            }
            List<StateTriggerRunner<T, E>> runners = new java.util.ArrayList<>();
            for (StatefulMechanismRuntimeConfig.StateTriggerBinding<T, E> binding : bindings) {
                if (binding == null || binding.state() == null || binding.trigger() == null) {
                    continue;
                }
                E state;
                try {
                    state = (E) binding.state();
                } catch (ClassCastException e) {
                    continue;
                }
                runners.add(new StateTriggerRunner<>(state, binding.trigger()));
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
                stateMachine.queue(trigger.target);
            }
        }

        public double getSetpoint() {
            if (queuedSetpoint != null) {
                return queuedSetpoint.doubleValue();
            }
            return resolveStateSetpoint(stateMachine.getGoalState());
        }

        private double resolveStateSetpoint(E state) {
            if (state == null) {
                return lastFiniteSetpoint;
            }
            Double setpoint = StateSpecAccess.setpoint(state);
            if (setpoint != null && Double.isFinite(setpoint)) {
                lastFiniteSetpoint = setpoint;
                return setpoint;
            }
            StateSeed<E> seed = seedFor(state);
            if (seed == null) {
                return lastFiniteSetpoint;
            }
            if (seed.kind() == StateSeed.Kind.SETPOINT && Double.isFinite(seed.setpoint())) {
                lastFiniteSetpoint = seed.setpoint();
                return seed.setpoint();
            }
            if (seed.kind() == StateSeed.Kind.DSL && seed.dsl() != null) {
                StateBuilder<E> builder = new StateBuilder<>();
                StateBuilder<E> applied = seed.dsl().apply(builder);
                StateBuilder<E> resolved = applied != null ? applied : builder;
                Double dslSetpoint = resolved.setpoint();
                if (dslSetpoint != null && Double.isFinite(dslSetpoint)) {
                    lastFiniteSetpoint = dslSetpoint;
                    return dslSetpoint;
                }
            }
            return lastFiniteSetpoint;
        }

        public void queueSetpoint(double setpoint) {
            if (!Double.isFinite(setpoint)) {
                return;
            }
            queuedSetpoints.add(setpoint);
        }

        public void forceSetpoint(double setpoint) {
            if (!Double.isFinite(setpoint)) {
                return;
            }
            stateMachine.resetQueue();
            queuedSetpoints.clear();
            queuedSetpoint = setpoint;
        }

        public void forceState(E target) {
            Objects.requireNonNull(target, "target");
            queuedSetpoints.clear();
            queuedSetpoint = null;
            stateMachine.force(target);
        }

        public void clear() {
            queuedSetpoints.clear();
            queuedSetpoint = null;
            stateMachine.resetQueue();
        }

        @SuppressWarnings("unchecked")
        public void updateMechanism(Mechanism mechanism) {
            update((T) mechanism);
        }

        public void update(T instance) {
            // Update the context with the current goal state so triggers can examine live mechanism data.
            context.update(instance, stateMachine.getGoalState(), getSetpoint());
            evaluateStateTriggers();
            stateMachine.update();
            updateQueuedSetpoint();
            E currentState = stateMachine.getGoalState();
            applyDslSeedBehavior(instance, currentState);
            instance.control().setpoint(getSetpoint());
            boolean changed = !Objects.equals(currentState, previousState);
            E from = previousState;
            if (!instance.areHooksEnabled()) {
                previousState = currentState;
                return;
            }
            if (changed) {
                applyExitHooks(instance, from);
            }
            context.update(instance, currentState, getSetpoint());
            if (changed) {
                applyTransitionHooks(instance, from, currentState);
                applyEnterHooks(currentState);
                previousState = currentState;
            }
            applyHooks(currentState);
            Function<T, Boolean> action = stateActions.get(currentState);
            if (action != null) {
                action.apply(instance);
            }
        }

        private void applyDslSeedBehavior(T instance, E state) {
            StateSeed<E> seed = seedFor(state);
            if (seed == null || seed.kind() != StateSeed.Kind.DSL || seed.dsl() == null) {
                clearDslManualOverride(instance);
                return;
            }

            StateBuilder<E> builder = new StateBuilder<>();
            StateBuilder<E> applied = seed.dsl().apply(builder);
            StateBuilder<E> resolved = applied != null ? applied : builder;

            Double manualPercent = resolved.manualPercent();
            if (manualPercent != null && Double.isFinite(manualPercent)) {
                instance.manualOverride(true);
                instance.speed(manualPercent);
                dslManualOverrideActive = true;
            } else {
                clearDslManualOverride(instance);
            }

            E nextState = resolved.next() != null ? resolved.next() : StateSpecAccess.resolve(state, resolved.nextName());
            if (resolved.until() == null || nextState == null) {
                return;
            }

            StateCtx<E> stateCtx = new StateCtx<>() {
                @Override
                public E state() {
                    return state;
                }

                @Override
                public double timeInState() {
                    return stateMachine.goalStateTimeSeconds();
                }

                @Override
                public Mechanism mechanism() {
                    return instance;
                }
            };

            boolean shouldQueue;
            try {
                shouldQueue = resolved.until().test(stateCtx);
            } catch (RuntimeException ex) {
                return;
            }
            if (shouldQueue
                    && !stateMachine.isGoalState(nextState)
                    && !stateMachine.isQueued(nextState)) {
                stateMachine.queue(nextState);
            }
        }

        private void clearDslManualOverride(T instance) {
            if (!dslManualOverrideActive) {
                return;
            }
            instance.manualOverride(false);
            dslManualOverrideActive = false;
        }

        private StateSeed<E> seedFor(E state) {
            return StateSpecAccess.seed(state);
        }

        private void updateQueuedSetpoint() {
            if (!stateMachine.atGoal()) {
                return;
            }
            if (queuedSetpoint == null) {
                if (!queuedSetpoints.isEmpty()) {
                    queuedSetpoint = queuedSetpoints.poll();
                }
                return;
            }
            queuedSetpoint = queuedSetpoints.poll();
        }

        private static final class StateTriggerRunner<T extends Mechanism, E> {
            private final E target;
            private final StatefulMechanismRuntimeConfig.StateTrigger<T, E> trigger;
            private boolean last;

            private StateTriggerRunner(E target, StatefulMechanismRuntimeConfig.StateTrigger<T, E> trigger) {
                this.target = target;
                this.trigger = trigger;
            }
        }

        private void applyEnterHooks(E state) {
            if (state == null) {
                return;
            }
            List<StatefulMechanismRuntimeConfig.StateHook<T, E>> hooks = enterStateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (StatefulMechanismRuntimeConfig.StateHook<T, E> binding : hooks) {
                applyHook(binding);
            }
        }

        private void applyTransitionHooks(T instance, E from, E to) {
            if (from == null || to == null || transitionHooks == null || transitionHooks.isEmpty()) {
                return;
            }
            for (StatefulMechanismRuntimeConfig.TransitionHookBinding<T, E> hook : transitionHooks) {
                if (hook == null) {
                    continue;
                }
                if (!Objects.equals(hook.from(), from) || !Objects.equals(hook.to(), to)) {
                    continue;
                }
                hook.hook().apply(context, from, to);
            }
        }

        private void applyExitHooks(T instance, E state) {
            if (state == null) {
                return;
            }
            context.update(instance, state, resolveStateSetpoint(state));
            for (StatefulMechanismRuntimeConfig.StateHook<T, E> binding : exitAlwaysHooks) {
                applyHook(binding);
            }
            List<StatefulMechanismRuntimeConfig.StateHook<T, E>> hooks = exitStateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (StatefulMechanismRuntimeConfig.StateHook<T, E> binding : hooks) {
                applyHook(binding);
            }
        }

        private void applyHooks(E state) {
            for (StatefulMechanismRuntimeConfig.StateHook<T, E> binding : alwaysHooks) {
                applyHook(binding);
            }
            List<StatefulMechanismRuntimeConfig.StateHook<T, E>> hooks = stateHooks.get(state);
            if (hooks == null) {
                return;
            }
            for (StatefulMechanismRuntimeConfig.StateHook<T, E> binding : hooks) {
                applyHook(binding);
            }
        }

        private void applyHook(StatefulMechanismRuntimeConfig.StateHook<T, E> binding) {
            binding.apply(context);
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
            private double setpoint;
            private final TypedInputResolver inputsView = new TypedInputResolver(
                    "StatefulMechanismContext",
                    TypedInputResolver.ValueMode.STRICT,
                    new TypedInputResolver.MutableInputs() {
                        @Override
                        public boolean hasBool(String key) {
                            return mechanism != null && mechanism.hasMutableBoolValKey(key);
                        }

                        @Override
                        public boolean bool(String key) {
                            return mechanism.mutableBoolVal(key);
                        }

                        @Override
                        public boolean hasDouble(String key) {
                            return mechanism != null && mechanism.hasMutableDblValKey(key);
                        }

                        @Override
                        public double dbl(String key) {
                            return mechanism.mutableDblVal(key);
                        }

                        @Override
                        public boolean hasInt(String key) {
                            return mechanism != null && mechanism.hasMutableIntValKey(key);
                        }

                        @Override
                        public int intVal(String key) {
                            return mechanism.mutableIntVal(key);
                        }

                        @Override
                        public boolean hasString(String key) {
                            return mechanism != null && mechanism.hasMutableStrValKey(key);
                        }

                        @Override
                        public String str(String key) {
                            return mechanism.mutableStrVal(key);
                        }

                        @Override
                        public boolean hasPose2d(String key) {
                            return mechanism != null && mechanism.hasMutablePose2dValKey(key);
                        }

                        @Override
                        public Pose2d pose2d(String key) {
                            return mechanism.mutablePose2dVal(key);
                        }

                        @Override
                        public boolean hasPose3d(String key) {
                            return mechanism != null && mechanism.hasMutablePose3dValKey(key);
                        }

                        @Override
                        public Pose3d pose3d(String key) {
                            return mechanism.mutablePose3dVal(key);
                        }
                    },
                    inputs,
                    doubleInputs,
                    intInputs,
                    stringInputs,
                    pose2dInputs,
                    pose3dInputs,
                    objectInputs);

            private void update(T mechanism, E state, double setpoint) {
                this.mechanism = mechanism;
                this.state = state;
                this.setpoint = setpoint;
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
            public double setpoint() {
                return setpoint;
            }

            @Override
            public boolean input(String key) {
                return inputsView.boolVal(key);
            }

            @Override
            public BooleanSupplier inputSupplier(String key) {
                return inputsView.boolSupplier(key);
            }

            @Override
            public double doubleInput(String key) {
                return inputsView.doubleVal(key);
            }

            @Override
            public DoubleSupplier doubleInputSupplier(String key) {
                return inputsView.doubleSupplier(key);
            }

            @Override
            public int intVal(String key) {
                return inputsView.intVal(key);
            }

            @Override
            public IntSupplier intValSupplier(String key) {
                return inputsView.intSupplier(key);
            }

            @Override
            public String stringVal(String key) {
                return inputsView.stringVal(key);
            }

            @Override
            public Supplier<String> stringValSupplier(String key) {
                return inputsView.stringSupplier(key);
            }

            @Override
            public Pose2d pose2dVal(String key) {
                return inputsView.pose2dVal(key);
            }

            @Override
            public Supplier<Pose2d> pose2dValSupplier(String key) {
                return inputsView.pose2dSupplier(key);
            }

            @Override
            public Pose3d pose3dVal(String key) {
                return inputsView.pose3dVal(key);
            }

            @Override
            public Supplier<Pose3d> pose3dValSupplier(String key) {
                return inputsView.pose3dSupplier(key);
            }

            @Override
            public <V> V objectInput(String key, Class<V> type) {
                return inputsView.objectVal(key, type);
            }

            @Override
            public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
                return inputsView.objectSupplier(key, type);
            }
        }
    }
}
