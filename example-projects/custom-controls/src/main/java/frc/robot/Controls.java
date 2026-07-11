package frc.robot;

import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.ControlSignal;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import edu.wpi.first.wpilibj.DriverStation;

public final class Controls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);

    public Controls(Robot robot) {
        operator.a().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(Actions.parallel(robot.arm.home, robot.flywheel.off));
        operator.x().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.arm.score);
        operator.y().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(Actions.parallel(
                        robot.arm.holdOperatorSetpoint,
                        robot.flywheel.withArbitraryFeedforward));

        ControlSignal arbitraryFeedforward = operator.rightBumper()
                .onlyIf(DriverStation::isTeleopEnabled);
        arbitraryFeedforward
                .whileTrue(robot.flywheel.withArbitraryFeedforward)
                .onFalse(robot.flywheel.off);

        ControlSignal devicePid = operator.leftBumper()
                .onlyIf(DriverStation::isTeleopEnabled);
        devicePid
                .whileTrue(robot.flywheel.withDevicePid)
                .onFalse(robot.flywheel.off);

        operator.b().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.advancedFeedback.modularAbsoluteMove);
        operator.povUp().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.advancedFeedback.ordinaryRelativeMove);
        operator.povRight().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.advancedFeedback.redundantMove);
        operator.povDown().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.advancedFeedback.stop);
    }
}
