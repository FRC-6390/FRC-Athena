package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.controllers.ArmFeedforwardSendable;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ArmMechanism extends Mechanism {
    private final ArmFeedforwardSendable feedforward;

    public ArmMechanism(MechanismConfig<? extends ArmMechanism> config, ArmFeedforward feedforward) {
        super(config);
        this.feedforward = new ArmFeedforwardSendable(feedforward.getKs(),feedforward.getKg(),feedforward.getKv(),feedforward.getKa());
        setFeedforwardEnabled(true);
    }

    @Override
    public double calculateFeedForward() {
        double value = feedforward.calculate(getControllerSetpointPosition(), getControllerSetpointVelocity());
        return  isUseVoltage() ? value : value / 12d;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        tab.add("FeedForwards", feedforward);
        return super.shuffleboard(tab);
    }

    @Override
    public ArmMechanism shuffleboard(String tab) {
        return (ArmMechanism) super.shuffleboard(tab);
    }
    
    public static class StatefulArmMechanism<E extends Enum<E> & SetpointProvider<Double>> extends ArmMechanism {

        private final StatefulMechanismCore<StatefulArmMechanism<E>, E> stateCore;

        public StatefulArmMechanism(MechanismConfig<StatefulArmMechanism<E>> config, ArmFeedforward feedforward, E initialState) {
            super(config, feedforward);
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

        @SuppressWarnings("unchecked")
        @Override
        public StatefulArmMechanism<E> shuffleboard(String tab) {
            return (StatefulArmMechanism<E>) super.shuffleboard(tab);
        }
    }
}



