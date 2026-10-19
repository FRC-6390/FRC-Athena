package frc.robot;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);
    public final Gamepad sysId = Controllers.xbox(Constants.Operator.SYSID_PORT);

    public Controls(Robot robot) {
        operator.a().onTrue(robot.superstructure.home);
        operator.x().onTrue(robot.superstructure.collectFloor);
        operator.y().onTrue(robot.superstructure.scoreHigh);
        operator.b().onTrue(robot.superstructure.ejectAll);

        operator.rightTrigger(0.5)
                .whileTrue(robot.openLoopShooter.shoot)
                .onFalse(robot.openLoopShooter.stop);
        operator.leftTrigger(0.5)
                .whileTrue(robot.indexedIntake.collect)
                .onFalse(robot.indexedIntake.stop);
        operator.rightBumper()
                .whileTrue(robot.limitedManualArm.raise)
                .onFalse(robot.limitedManualArm.stop);
        operator.leftBumper()
                .whileTrue(robot.limitedManualArm.lower)
                .onFalse(robot.limitedManualArm.stop);

        operator.povUp().onTrue(robot.followerElevator.top);
        operator.povRight().onTrue(robot.singleJointArm.score);
        operator.povDown().onTrue(robot.followerElevator.bottom);
        operator.povLeft().onTrue(robot.singleJointArm.pickup);

        // Start runs the repeating cycle; Back replaces it with stow on the same mechanism root.
        operator.start().onTrue(robot.singleJointArm.exercise);
        operator.back().onTrue(robot.singleJointArm.stow);
        operator.leftStick().onTrue(robot.turret.forward);
        operator.rightStick().onTrue(robot.turret.rear);

        operator.leftY().above(0.75, 0.65)
                .whileTrue(robot.templateRoller.run)
                .onFalse(robot.templateRoller.stop);
        operator.leftY().below(-0.75, -0.65)
                .whileTrue(robot.shooter.distanceShot)
                .onFalse(robot.shooter.stop);

        sysId.a().whileHeld(robot.turret.sysId.quasistaticForward()).onRelease(robot.turret.neutral);
        sysId.b().whileHeld(robot.turret.sysId.quasistaticReverse()).onRelease(robot.turret.neutral);
        sysId.x().whileHeld(robot.turret.sysId.dynamicForward()).onRelease(robot.turret.neutral);
        sysId.y().whileHeld(robot.turret.sysId.dynamicReverse()).onRelease(robot.turret.neutral);
    }
}
