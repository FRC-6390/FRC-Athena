package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import ca.frc6390.athena.wpilib.telemetry.MechanismTracePublisher;
import frc.robot.examples.FieldTelemetry;
import frc.robot.mechanisms.FollowerElevator;
import frc.robot.mechanisms.IndexedIntake;
import frc.robot.mechanisms.LimitedManualArm;
import frc.robot.mechanisms.OpenLoopShooter;
import frc.robot.mechanisms.PositionElevator;
import frc.robot.mechanisms.RollerIntake;
import frc.robot.mechanisms.SingleJointArm;
import frc.robot.mechanisms.SplitWheelShooter;
import frc.robot.mechanisms.TemplateRoller;
import frc.robot.mechanisms.Superstructure;
import frc.robot.mechanisms.TwoJointArm;
import frc.robot.mechanisms.Turret;
import frc.robot.mechanisms.VelocityShooter;

public final class Robot extends AthenaRobot {
    public final VelocityShooter shooter = new VelocityShooter();
    public final TwoJointArm arm = new TwoJointArm();
    public final RollerIntake intake = new RollerIntake();
    public final PositionElevator elevator = new PositionElevator();
    public final Superstructure superstructure = new Superstructure(shooter, arm, intake, elevator);

    public final OpenLoopShooter openLoopShooter = new OpenLoopShooter();
    public final SplitWheelShooter splitWheelShooter = new SplitWheelShooter();
    public final SingleJointArm singleJointArm = new SingleJointArm();
    public final LimitedManualArm limitedManualArm = new LimitedManualArm();
    public final IndexedIntake indexedIntake = new IndexedIntake();
    public final FollowerElevator followerElevator = new FollowerElevator();
    public final Turret turret = new Turret();
    public final TemplateRoller templateRoller = new TemplateRoller().motor.fill(
            Constants.RIO.motor(MotorControllerKinds.SPARK_FLEX, MotorKinds.NEO, 15));
    public final FieldTelemetry fieldTelemetry = new FieldTelemetry();
    public final Controls controls = new Controls(this);

    public Robot() {
        traceTelemetry(MechanismTracePublisher.Profile.CAPTURE);
    }
}
