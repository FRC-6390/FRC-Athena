package ca.frc6390.athena.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismGroup extends SubsystemBase {

    public record MechanismGroupConfig(MechanismConfig<? extends Mechanism>[] configs){

    }

    public MechanismGroup(MechanismConfig<? extends Mechanism>[] configs){

    }
}
