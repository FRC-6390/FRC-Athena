package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwerveDriveCharacterization;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Drivetrain implements Mechanism {
    /** Battery share: when the budget runs short, everything else gives up current before the drivetrain. */
    private static final PowerProfile DRIVE = PowerProfile.of()
            .sustained(Amps.of(60))
            .share(4.0);


    private static final Distance WHEELBASE = Meters.of(0.55);
    private static final Distance TRACK_WIDTH = Meters.of(0.55);
    private static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.0);
    private static final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(Math.PI);


    // Devices
    private final ImuDevice imu = Constants.CAN.imu(ImuKinds.PIGEON_2, 0);
    private final ImuSource imuSource = imu.relative();
    private final ImuSource driverHeading = imuSource.relative();
    private final SwerveModule frontLeft = new SwerveModules.SDS.MK4I.L3().drive
            .fill(Constants.CAN.motor(MotorKinds.KRAKEN_X60, 1).brake().inverted().currentLimit(Amps.of(60)).power(DRIVE)).steer
            .fill(Constants.CAN.motor(MotorKinds.FALCON_500, 5).inverted().currentLimit(Amps.of(50))).angle
            .fill(Constants.CAN.encoder(EncoderKinds.CANCODER, 9)
                    .measures(Rotations).offset(0.384))
            .driveMaxSpeed(MAX_SPEED)
            .steerPid(14, 0.0, 0.0);

    private final SwerveModule frontRight = new SwerveModules.SDS.MK4I.L3().drive
            .fill(Constants.CAN.motor(MotorKinds.KRAKEN_X60, 2).brake().inverted().currentLimit(Amps.of(60)).power(DRIVE)).steer
            .fill(Constants.CAN.motor(MotorKinds.FALCON_500, 6).inverted().currentLimit(Amps.of(50))).angle
            .fill(Constants.CAN.encoder(EncoderKinds.CANCODER, 10)
                    .measures(Rotations).offset(0.171))
            .driveMaxSpeed(MAX_SPEED)
            .steerPid(14, 0.0, 0.0);
    private final SwerveModule backLeft = new SwerveModules.SDS.MK4I.L3().drive
            .fill(Constants.CAN.motor(MotorKinds.KRAKEN_X60, 3).brake().inverted().currentLimit(Amps.of(60)).power(DRIVE)).steer
            .fill(Constants.CAN.motor(MotorKinds.FALCON_500, 7).inverted().currentLimit(Amps.of(50))).angle
            .fill(Constants.CAN.encoder(EncoderKinds.CANCODER, 11)
                    .measures(Rotations).offset(0.693))
            .driveMaxSpeed(MAX_SPEED)
            .steerPid(14, 0.0, 0.0);
    private final SwerveModule backRight = new SwerveModules.SDS.MK4I.L3().drive
            .fill(Constants.CAN.motor(MotorKinds.KRAKEN_X60, 4).brake().inverted().currentLimit(Amps.of(60)).power(DRIVE)).steer
            .fill(Constants.CAN.motor(MotorKinds.FALCON_500, 8).inverted().currentLimit(Amps.of(50))).angle
            .fill(Constants.CAN.encoder(EncoderKinds.CANCODER, 12)
                    .measures(Rotations).offset(0.365))
            .driveMaxSpeed(MAX_SPEED)
            .steerPid(14, 0.0, 0.0);

    // Bindings and controls
    private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
            WHEELBASE,
            TRACK_WIDTH,
            MAX_SPEED,
            frontLeft,
            frontRight,
            backLeft,
            backRight);
    public final SwerveOdometry odometry = kinematics.odometry(imuSource);
    private final RobotVelocityPool velocity = new RobotVelocityPool();
    private final RobotVelocityPool.Channel driverVelocity = velocity.channel()
            .enabled(DriverStation::isTeleopEnabled);
    public final RobotVelocityPool.Channel autoVelocity = velocity.channel()
            .enabled(DriverStation::isAutonomousEnabled);
    private final RobotVelocityPool.AngularChannel aimVelocity = velocity.angularChannel();
    private volatile Supplier<Pose2d> aimingPose = Pose2d::new;
    // Set by Localization once it exists (Drivetrain is constructed first).
    private volatile Supplier<Pose2d> fieldPose;
    private final ControlBinding<?> aimHeading = Controls.position(aimVelocity, Radians)
            .feedback(() -> aimingPose.get().getRotation().getRadians())
            .pid(3, 0.0, 0.0)
            .continuous(Radians.of(-Math.PI), Radians.of(Math.PI))
            .constraint(Constraints.maxOutput(MAX_ROTATION));

    // Hooks
    public final HookBinding alignDriverHeadingAfterAuto = Events.autonomousExit()
            .onStart(driverHeading.setYaw(this::allianceDriverHeading));

    // Actions
    public final Action resetDriverHeading = driverHeading.setYaw(Degrees.of(0.0));
    public final Action pooledDrive = kinematics.drive(velocity, () -> new Rotation2d(driverHeading.yaw()));

    /** Whole-drivetrain characterization: all modules point straight and get the same drive voltage. */
    public final SwerveDriveCharacterization driveCharacterization = kinematics.characterization()
            .maxVoltage(Volts.of(6.0))
            .rampRate(Volts.of(1.0).per(Seconds))
            .timeout(Seconds.of(8.0));

    public Action drive(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier fieldOriented) {
        driverVelocity.set(() -> {
            LinearVelocity x = MAX_SPEED.times(clamp(forward.getAsDouble()));
            LinearVelocity y = MAX_SPEED.times(clamp(strafe.getAsDouble()));
            AngularVelocity omega = MAX_ROTATION.times(clamp(rotation.getAsDouble()));
            return fieldOriented.getAsBoolean()
                    ? RobotVelocity.field(x, y, omega)
                    : RobotVelocity.robot(x, y, omega);
        });
        return pooledDrive;
    }

    /** Preserves pooled translation while pointing the rear-mounted shooter at the alliance hub. */
    public Action aimAtHub(Supplier<Pose2d> robotPose) {
        Supplier<Pose2d> safePose = Objects.requireNonNull(robotPose, "robotPose");
        aimingPose = () -> Objects.requireNonNull(
                safePose.get(),
                "robot pose supplier returned null");
        // Aiming carries its own pooled drive so the wheels follow the heading request even
        // when no other action (driver hook, path follower) is running pooledDrive.
        return Actions.parallel(
                aimHeading.position(() -> Constants.Field.rearFacingHubHeadingRadians(
                        aimingPose.get()), Radians),
                pooledDrive);
    }

    public Action stopAiming(){
        return aimHeading.neutral();
    }

    /** Supplies the localized field pose used to re-align field-oriented driving after auto. */
    public void fieldPose(Supplier<Pose2d> pose) {
        fieldPose = Objects.requireNonNull(pose, "pose");
    }

    /** Driver-forward points away from the alliance driver station. */
    private Angle allianceDriverHeading() {
        Supplier<Pose2d> pose = fieldPose;
        Pose2d current = pose == null ? null : pose.get();
        if (current == null) {
            return driverHeading.yaw();
        }
        Angle heading = current.getRotation().getMeasure();
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? heading.plus(Degrees.of(180.0))
                : heading;
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    
    



 
    }


