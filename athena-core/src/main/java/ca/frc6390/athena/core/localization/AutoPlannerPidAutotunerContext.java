package ca.frc6390.athena.core.localization;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface AutoPlannerPidAutotunerContext {
    RobotLocalization<?> localization();

    Subsystem requirement();

    String dashboardPath();

    String outputSource();

    HolonomicPidConstants translationPid();

    HolonomicPidConstants rotationPid();

    void output(ChassisSpeeds speeds);

    void stopOutput();

    default Command relayTheta() {
        return AutoPlannerPidAutotuners.relayTheta(this);
    }

    default Command relayTheta(AutoPlannerPidAutotuners.RelayThetaConfig config) {
        return AutoPlannerPidAutotuners.relayTheta(this, config);
    }
}
