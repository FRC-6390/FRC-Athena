package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class StateMachineMechanism<E extends Enum<E> & SetpointProvider> extends ProfiledMechanism {
    
    public StateMachine<E> stateMachine;

    public StateMachineMechanism(MotorController motorController, ProfiledPIDController controller){
        super(motorController, controller);
    }

    public StateMachineMechanism(MotorController motorController, Encoder encoder, ProfiledPIDController controller){
        super(motorController, encoder, controller);
    }


}
