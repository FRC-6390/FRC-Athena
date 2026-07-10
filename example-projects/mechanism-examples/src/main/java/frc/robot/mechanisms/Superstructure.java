package frc.robot.mechanisms;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;

public final class Superstructure implements Mechanism {
    private final VelocityShooter shooter;
    private final TwoJointArm arm;
    private final RollerIntake intake;
    private final PositionElevator elevator;

    public final Action home;
    public final Action collectFloor;
    public final Action scoreHigh;
    public final Action ejectAll;

    public Superstructure(
            VelocityShooter shooter,
            TwoJointArm arm,
            RollerIntake intake,
            PositionElevator elevator) {
        this.shooter = shooter;
        this.arm = arm;
        this.intake = intake;
        this.elevator = elevator;
        home = Actions.parallel(shooter.stop, arm.stow, intake.stop, elevator.home);
        collectFloor = Actions.parallel(shooter.stop, arm.floorPickup, intake.intake, elevator.low);
        scoreHigh = Actions.sequence()
                .forTime(0.75, Actions.parallel(shooter.podium, arm.trapScore, elevator.high))
                .then(Actions.parallel(shooter.podium, arm.trapScore, intake.eject, elevator.high));
        ejectAll = Actions.parallel(shooter.reverse, arm.ampScore, intake.eject, elevator.low);
    }
}
