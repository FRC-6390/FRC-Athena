package frc.robot;

import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import frc.robot.mechanisms.AdvancedFeedback;
import frc.robot.mechanisms.ExperimentalArm;
import frc.robot.mechanisms.Flywheel;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(ExperimentalArm arm, Flywheel flywheel, AdvancedFeedback advancedFeedback) {
        operator.a().onActive(Actions.parallel(arm.home, flywheel.off));
        operator.x().onActive(arm.score);
        operator.y().onActive(Actions.parallel(
                arm.holdOperatorSetpoint,
                flywheel.withArbitraryFeedforward));
        operator.rightBumper()
                .whileActive(flywheel.withArbitraryFeedforward)
                .onDeactive(flywheel.off);
        operator.leftBumper()
                .whileActive(flywheel.withDevicePid)
                .onDeactive(flywheel.off);
        operator.b().onActive(advancedFeedback.modularAbsoluteMove);
        operator.povUp().onActive(advancedFeedback.ordinaryRelativeMove);
        operator.povRight().onActive(advancedFeedback.redundantMove);
        operator.povDown().onActive(advancedFeedback.stop);
    }
}
