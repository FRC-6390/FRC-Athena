package frc.robot;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Swerve sensor layout used by the localization examples. */
public final class DriveTrain implements Mechanism {
    private static final double MAX_SPEED_METERS_PER_SECOND = 4.0;
    private static final double MAX_ROTATION_RADIANS_PER_SECOND = Math.PI;

    public final SwerveModule frontLeft = module(1, 2, 11, Constants.ModuleOffsets.FRONT_LEFT);
    public final SwerveModule frontRight = module(3, 4, 12, Constants.ModuleOffsets.FRONT_RIGHT);
    public final SwerveModule backLeft = module(5, 6, 13, Constants.ModuleOffsets.BACK_LEFT);
    public final SwerveModule backRight = module(7, 8, 14, Constants.ModuleOffsets.BACK_RIGHT);
    public final ImuDevice imu = Constants.RIO.imu(ImuKinds.PIGEON_2, 20);
    public final ImuSource heading = imu.relative();
    public final SwerveKinematics kinematics = SwerveKinematics.rectangular(
            0.55,
            0.55,
            MAX_SPEED_METERS_PER_SECOND,
            frontLeft,
            frontRight,
            backLeft,
            backRight);
    public final SwerveOdometry odometry = kinematics.odometry(heading);
    public final RobotVelocityPool velocity = new RobotVelocityPool();
    public final RobotVelocityPool.Channel driverVelocity = velocity.channel()
            .enabled(DriverStation::isTeleopEnabled);
    public final RobotVelocityPool.Channel targetingVelocity = velocity.channel()
            .enabled(DriverStation::isTeleopEnabled);
    private DoubleSupplier fieldHeadingRadians = () -> Math.toRadians(heading.yawDegrees());
    public final Action pooledDrive = kinematics.drive(
            velocity,
            () -> fieldHeadingRadians.getAsDouble());

    public Action drive(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            BooleanSupplier fieldOriented,
            DoubleSupplier fieldHeadingRadians) {
        this.fieldHeadingRadians = fieldHeadingRadians;
        driverVelocity.set(() -> {
            RobotVelocity requested = RobotVelocity.robot(
                    linearSpeed(forward.getAsDouble()),
                    linearSpeed(strafe.getAsDouble()),
                    rotationSpeed(rotation.getAsDouble()));
            if (fieldOriented.getAsBoolean()) {
                requested = RobotVelocity.field(
                        requested.xMetersPerSecond(),
                        requested.yMetersPerSecond(),
                        requested.angularRadiansPerSecond());
            }
            return requested;
        });
        return pooledDrive;
    }

    private static SwerveModule module(
            int driveMotorId,
            int steerMotorId,
            int encoderId,
            double angleOffsetRotations) {
        return new SwerveModules.SDS.MK5N.R3()
                .drive.brake().fill(Constants.RIO.motor(MotorKinds.KRAKEN_X60, driveMotorId))
                .steer.brake().fill(Constants.RIO.motor(MotorKinds.KRAKEN_X44, steerMotorId))
                .angle.fill(Constants.RIO.encoder(EncoderKinds.CANCODER, encoderId)
                        .offset(angleOffsetRotations)
                        .units(EncoderUnit.ROTATIONS))
                .driveMaxSpeedMetersPerSecond(MAX_SPEED_METERS_PER_SECOND)
                .steerPid(22.8, 0.0, 0.0);
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private static double linearSpeed(double input) {
        return clamp(input) * MAX_SPEED_METERS_PER_SECOND;
    }

    private static double rotationSpeed(double input) {
        return clamp(input) * MAX_ROTATION_RADIANS_PER_SECOND;
    }
}
