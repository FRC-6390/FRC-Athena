package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.arcp.ArcpPolicy;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.mechanism.persist.Locks;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.Retention;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import ca.frc6390.athena.wpilib.telemetry.MechanismTracePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.examples.FieldTelemetry;
import frc.robot.mechanisms.FollowerElevator;
import frc.robot.mechanisms.IndexedIntake;
import frc.robot.mechanisms.LimitedManualArm;
import frc.robot.mechanisms.OpenLoopShooter;
import frc.robot.mechanisms.PositionElevator;
import frc.robot.mechanisms.RollerIntake;
import frc.robot.mechanisms.SingleJointArm;
import frc.robot.mechanisms.SplitWheelShooter;
import frc.robot.mechanisms.StagedArm;
import frc.robot.mechanisms.TemplateRoller;
import frc.robot.mechanisms.Superstructure;
import frc.robot.mechanisms.TwoJointArm;
import frc.robot.mechanisms.Turret;
import frc.robot.mechanisms.VelocityShooter;

public final class Robot extends AthenaRobot {
    /** Replay: 48 MiB buffer, files written on match end, disable after a match, and brownout recovery. */
    public final ReplayPolicy replay = ReplayPolicy.of()
            .stream(false)
            .bufferMegabytes(48)
            .flushOn(ReplayPolicy.Flush.MATCH_END,
                    ReplayPolicy.Flush.DISABLE_AFTER_MATCH,
                    ReplayPolicy.Flush.BROWNOUT_RECOVERY)
            .retention(Retention.maxTotalMegabytes(400)
                    .maxFiles(60)
                    .keepMatchesOverPractice()
                    .minimumFreeDiskMegabytes(150));

    /** ARCP on the default ports (TCP 5808, UDP 5809). Use ArcpPolicy.disabled() to turn it off completely. */
    public final ArcpPolicy arcp = ArcpPolicy.of()
            .bandwidthBudgetBytesPerSecond(400_000);

    /** Battery floor and start staggering. Without this field PowerPolicy.defaults() applies. */
    public final PowerPolicy power = PowerPolicy.defaults()
            .floor(Volts.of(7.5))
            .staggerStarts(Seconds.of(0.2))
            .estimateBatteryResistance(true);

    public final VelocityShooter shooter = new VelocityShooter();
    public final TwoJointArm arm = new TwoJointArm();
    public final RollerIntake intake = new RollerIntake();
    public final PositionElevator elevator = new PositionElevator();
    public final Superstructure superstructure = new Superstructure(shooter, arm, intake, elevator);

    public final OpenLoopShooter openLoopShooter = new OpenLoopShooter();
    public final SplitWheelShooter splitWheelShooter = new SplitWheelShooter();
    public final SingleJointArm singleJointArm = new SingleJointArm();
    public final StagedArm stagedArm = new StagedArm();
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

        // Calibration values can be written only while the robot is disabled.
        tuningLock(Calibration.class, () -> DriverStation.isEnabled());
        // The default group, shown explicitly: tuning is locked while FMS is attached.
        tuningLock(Locks.InMatch.class, DriverStation::isFMSAttached);
    }
}
