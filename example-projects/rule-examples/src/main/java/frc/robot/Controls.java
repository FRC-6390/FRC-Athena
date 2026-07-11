package frc.robot;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(Robot robot) {
        operator.a().onTrue(robot.arm.homeSlowly);
        operator.y().onTrue(robot.arm.score);
        operator.x().onTrue(robot.arm.timedManualEscape);
        operator.rightBumper()
                .whileTrue(robot.arm.nudgeUp)
                .onFalse(robot.arm.stop);

        operator.b().onTrue(robot.conveyor.indexOneBall);
        operator.leftBumper()
                .whileTrue(robot.conveyor.feedWhenLoaded)
                .onFalse(robot.conveyor.stop);
    }
}
