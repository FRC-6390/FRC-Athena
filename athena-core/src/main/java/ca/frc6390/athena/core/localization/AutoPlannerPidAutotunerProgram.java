package ca.frc6390.athena.core.localization;

import edu.wpi.first.wpilibj2.command.Command;

@FunctionalInterface
public interface AutoPlannerPidAutotunerProgram {
    Command build(AutoPlannerPidAutotunerContext ctx);
}
