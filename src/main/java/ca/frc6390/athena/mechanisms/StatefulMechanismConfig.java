package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorControllerType;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class StatefulMechanismConfig<T extends StatefulMechanism<?>> extends MechanismConfig<StatefulMechanism> {

    public ArrayList<MotorControllerConfig> motors = new ArrayList<>();
    public EncoderConfig encoder = null;
    public PIDController pidController = null;
    public ProfiledPIDController profiledPIDController = null;
    public boolean useAbsolute = false;
    public boolean useVoltage = false;
    public Function<MechanismConfig<T>, T> factory = null;
    public ArrayList<GenericLimitSwitchConfig> limitSwitches = new ArrayList<>();
    public String canbus = null;

    
}
