package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.function.Function;

import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

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
