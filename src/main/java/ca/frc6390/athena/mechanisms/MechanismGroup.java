package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.Mechanism.MechanismConfig;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MechanismGroup implements Subsystem{
    
    public record MechanismGroupConfig(MechanismConfig<? extends Mechanism>[] configs){

    }

    public MechanismGroup(MechanismConfig<? extends Mechanism>[] configs){

    }
}
