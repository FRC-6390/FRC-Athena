package frc.robot;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import frc.robot.mechanisms.Conveyor;
import frc.robot.mechanisms.GuardedArm;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(GuardedArm arm, Conveyor conveyor) {
        operator.a().onActive(arm.homeSlowly);
        operator.y().onActive(arm.score);
        operator.x().onActive(arm.timedManualEscape);
        operator.rightBumper()
                .whileActive(arm.nudgeUp)
                .onDeactive(arm.hold);

        operator.b().onActive(conveyor.indexOneBall);
        operator.leftBumper()
                .whileActive(conveyor.feedWhenLoaded)
                .onDeactive(conveyor.stop);
    }
}
