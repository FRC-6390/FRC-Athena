package frc.robot.auto;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.FollowerBackend;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwervePathFollower;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import edu.wpi.first.math.geometry.Pose2d;

/** Minimal swerve mechanism showing the kinematics-owned path API. */
public final class ExampleDrive implements Mechanism {
    private static final double MAX_SPEED = 4.0;
    private Pose2d pose = new Pose2d();

    public final SwerveModule frontLeft = module(1, 5, 9);
    public final SwerveModule frontRight = module(2, 6, 10);
    public final SwerveModule backLeft = module(3, 7, 11);
    public final SwerveModule backRight = module(4, 8, 12);
    public final SwerveKinematics kinematics = SwerveKinematics.rectangular(
            0.55, 0.55, MAX_SPEED, frontLeft, frontRight, backLeft, backRight);
    public final RobotVelocityPool velocity = new RobotVelocityPool();
    public final RobotVelocityPool.Channel autoVelocity = velocity.channel();
    public final Action pooledDrive = kinematics.drive(
            velocity,
            () -> pose.getRotation().getRadians());
    public final SwervePathFollower pathFollower = kinematics.follow(
            FollowerBackend.CHOREO,
            this::pose,
            this::resetPose,
            PidGains.of(4.0, 0.0, 0.0),
            PidGains.of(3.0, 0.0, 0.0))
            .pooled(autoVelocity, pooledDrive);

    public Pose2d pose() {
        return pose;
    }

    public Action resetPose(Pose2d nextPose) {
        return Actions.doOnce(() -> pose = nextPose);
    }

    private static SwerveModule module(int driveId, int steerId, int encoderId) {
        return new SwerveModules.SDS.MK5N.R3()
                .drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId).brake())
                .steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId).brake())
                .angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId)
                        .units(EncoderUnit.ROTATIONS))
                .driveMaxSpeedMetersPerSecond(MAX_SPEED)
                .steerPid(12.0, 0.0, 0.0);
    }
}
