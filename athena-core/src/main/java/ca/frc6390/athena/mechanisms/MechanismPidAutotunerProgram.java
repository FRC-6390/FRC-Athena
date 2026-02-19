package ca.frc6390.athena.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;

@FunctionalInterface
public interface MechanismPidAutotunerProgram {
    Command build(MechanismPidAutotunerContext ctx);
}
