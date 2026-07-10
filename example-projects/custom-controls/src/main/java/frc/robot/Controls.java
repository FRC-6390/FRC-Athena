package frc.robot;

import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import frc.robot.mechanisms.ExperimentalArm;
import frc.robot.mechanisms.Flywheel;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(ExperimentalArm arm, Flywheel flywheel) {
        operator.a().onActive(Actions.parallel(arm.home, flywheel.off));
        operator.x().onActive(arm.motionProfiled);
        operator.y().onActive(Actions.parallel(
                arm.holdOperatorSetpoint,
                flywheel.withArbitraryFeedforward));
        operator.rightBumper()
                .whileActive(flywheel.withArbitraryFeedforward)
                .onDeactive(flywheel.off);
        operator.leftBumper()
                .whileActive(flywheel.withDevicePid)
                .onDeactive(flywheel.off);
    }
}
