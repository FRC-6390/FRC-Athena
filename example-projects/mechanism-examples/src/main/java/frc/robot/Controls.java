package frc.robot;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import frc.robot.mechanisms.FollowerElevator;
import frc.robot.mechanisms.IndexedIntake;
import frc.robot.mechanisms.LimitedManualArm;
import frc.robot.mechanisms.OpenLoopShooter;
import frc.robot.mechanisms.SingleJointArm;
import frc.robot.mechanisms.Superstructure;
import frc.robot.mechanisms.Turret;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(
            Superstructure superstructure,
            OpenLoopShooter shooter,
            SingleJointArm arm,
            LimitedManualArm manualArm,
            IndexedIntake intake,
            FollowerElevator elevator,
            Turret turret) {
        operator.a().onActive(superstructure.home);
        operator.x().onActive(superstructure.collectFloor);
        operator.y().onActive(superstructure.scoreHigh);
        operator.b().onActive(superstructure.ejectAll);

        operator.rightTrigger(0.5)
                .whileActive(shooter.shoot)
                .onDeactive(shooter.stop);
        operator.leftTrigger(0.5)
                .whileActive(intake.collect)
                .onDeactive(intake.stop);
        operator.rightBumper()
                .whileActive(manualArm.raise)
                .onDeactive(manualArm.stop);
        operator.leftBumper()
                .whileActive(manualArm.lower)
                .onDeactive(manualArm.stop);

        operator.povUp().onActive(elevator.top);
        operator.povRight().onActive(arm.score);
        operator.povDown().onActive(elevator.bottom);
        operator.povLeft().onActive(arm.pickup);

        // Start runs the repeating cycle; Back replaces it with stow on the same mechanism root.
        operator.start().onActive(arm.exercise);
        operator.back().onActive(arm.stow);
        operator.leftStick().onActive(turret.forward);
        operator.rightStick().onActive(turret.rear);
    }
}
