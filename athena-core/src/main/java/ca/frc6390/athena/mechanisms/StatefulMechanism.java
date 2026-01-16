package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class StatefulMechanism <E extends Enum<E> & SetpointProvider<Double>> extends Mechanism implements StatefulLike<E> {
        
    private final StatefulMechanismCore<StatefulMechanism<E>, E> stateCore;

    public StatefulMechanism(MechanismConfig<StatefulMechanism<E>> config, E initialState) {
        super(config);
        stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.data().stateMachineDelay(),
                config.stateActions, config.stateHooks, config.alwaysHooks, config.inputs, config.doubleInputs, config.objectInputs);
    }

    @Override
    public double getSetpoint() {
        return stateCore.getSetpoint();
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

    @Override
    @SuppressWarnings("unchecked")
    public StatefulMechanism<E> shuffleboard(String tab) {
        return (StatefulMechanism<E>) super.shuffleboard(tab);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        stateCore.shuffleboard(tab,level);
        return super.shuffleboard(tab,level);
    }

    @Override
    @SuppressWarnings("unchecked")
    public StatefulMechanism<E> shuffleboard(String tab, SendableLevel level) {
        return (StatefulMechanism<E>) super.shuffleboard(tab, level);
    }

    public static class StatefulMechanismCore<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        private final StateMachine<Double, E> stateMachine;
        private final Map<Enum<?>, Function<T, Boolean>> stateActions;
        private final Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> stateHooks;
        private final List<MechanismConfig.MechanismBinding<T, ?>> alwaysHooks;
        private final Map<String, BooleanSupplier> inputs;
        private final Map<String, DoubleSupplier> doubleInputs;
        private final Map<String, Supplier<?>> objectInputs;
        private final MechanismContextImpl context = new MechanismContextImpl();
        private DoubleSupplier setpointOverride;
        private BooleanSupplier outputSuppressor;

        public StatefulMechanismCore(E initialState, Supplier<Boolean> atSetpointSupplier, double delay,
                                        Map<Enum<?>, Function<T, Boolean>> stateActions,
                                        Map<Enum<?>, List<MechanismConfig.MechanismBinding<T, ?>>> stateHooks,
                                        List<MechanismConfig.MechanismBinding<T, ?>> alwaysHooks,
                                        Map<String, BooleanSupplier> inputs,
                                        Map<String, DoubleSupplier> doubleInputs,
                                        Map<String, Supplier<?>> objectInputs) {
            this.stateMachine = new StateMachine<>(initialState, atSetpointSupplier::get);
            stateMachine.setAtStateDelay(delay);
            this.stateActions = stateActions;
            this.stateHooks = stateHooks;
            this.alwaysHooks = alwaysHooks;
            this.inputs = inputs;
            this.doubleInputs = doubleInputs;
            this.objectInputs = objectInputs;
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
            stateMachine.update();
            E currentState = stateMachine.getGoalState();
            context.update(instance, currentState, getBaseSetpoint());
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

        public void shuffleboard(ShuffleboardTab tab, SendableLevel level) {
            stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList), level);
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
                BooleanSupplier supplier = inputs.get(key);
                return supplier != null && supplier.getAsBoolean();
            }

            @Override
            public BooleanSupplier inputSupplier(String key) {
                BooleanSupplier supplier = inputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No input found for key " + key);
                }
                return supplier;
            }

            @Override
            public double doubleInput(String key) {
                DoubleSupplier supplier = doubleInputs.get(key);
                return supplier != null ? supplier.getAsDouble() : Double.NaN;
            }

            @Override
            public DoubleSupplier doubleInputSupplier(String key) {
                DoubleSupplier supplier = doubleInputs.get(key);
                if (supplier == null) {
                    throw new IllegalArgumentException("No double input found for key " + key);
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
