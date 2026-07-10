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
import ca.frc6390.athena.mechanism.core.Mechanism;

/** Swerve sensor layout used by the localization examples. */
public final class DriveTrain implements Mechanism {
    private static final double MAX_SPEED_METERS_PER_SECOND = 4.0;

    public final SwerveModule frontLeft = module(1, 2, 11);
    public final SwerveModule frontRight = module(3, 4, 12);
    public final SwerveModule backLeft = module(5, 6, 13);
    public final SwerveModule backRight = module(7, 8, 14);
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

    private static SwerveModule module(int driveMotorId, int steerMotorId, int encoderId) {
        return new SwerveModules.SDS.MK5N.R3()
                .drive.brake().fill(Constants.RIO.motor(MotorKinds.KRAKEN_X60, driveMotorId))
                .steer.brake().fill(Constants.RIO.motor(MotorKinds.KRAKEN_X44, steerMotorId))
                .angle.fill(Constants.RIO.encoder(EncoderKinds.CANCODER, encoderId)
                        .units(EncoderUnit.ROTATIONS))
                .driveMaxSpeedMetersPerSecond(MAX_SPEED_METERS_PER_SECOND)
                .steerPid(1.9, 0.0, 0.0);
    }
}
