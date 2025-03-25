package ca.frc6390.athena.mechanisms;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class StatefulMechanism <E extends Enum<E> & SetpointProvider<Double>> extends Mechanism {
        
    private final StatefulMechanismCore<StatefulMechanism<E>, E> stateCore;

    public StatefulMechanism(MechanismConfig<StatefulMechanism<E>> config, E initialState) {
        super(config);
        stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay, config.stateActions);
    }

    @Override
    public double getSetpoint() {
        return stateCore.getSetpoint();
    }

    @Override
    public void update() {
        setSuppressMotorOutput(!stateCore.update(this));
        super.update();
    }

    public StateMachine<Double, E> getStateMachine() {
        return stateCore.getStateMachine();
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        stateCore.shuffleboard(tab);
        return super.shuffleboard(tab);
    }

    

    public static class StatefulMechanismCore<T, E extends Enum<E> & SetpointProvider<Double>> {
        private final StateMachine<Double, E> stateMachine;
        private final Map<Enum<?>, Function<T, Boolean>> stateActions;

        public StatefulMechanismCore(E initialState, Supplier<Boolean> atSetpointSupplier, double delay,
                                        Map<Enum<?>, Function<T, Boolean>> stateActions) {
            this.stateMachine = new StateMachine<>(initialState, atSetpointSupplier::get);
            stateMachine.setAtStateDelay(delay);
            this.stateActions = stateActions;
        }

        public double getSetpoint() {
            return stateMachine.getGoalState().getSetpoint();
        }

        public boolean update(T instance) {
            stateMachine.update();
            E currentState = stateMachine.getGoalState();
            Function<T, Boolean> action = stateActions.get(currentState);
            if (action != null) {
                return action.apply(instance);
            }
            return true;
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateMachine;
        }

        public void shuffleboard(ShuffleboardTab tab) {
            stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList));
        }
    }
}


