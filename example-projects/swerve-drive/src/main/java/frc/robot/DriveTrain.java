package frc.robot;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleTarget;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class DriveTrain implements Mechanism {
    private static final double MAX_SPEED_METERS_PER_SECOND = 4.0;
    private static final double MAX_ROTATION_RADIANS_PER_SECOND = Math.PI;
    private static final double WHEELBASE_METERS = 0.55;
    private static final double TRACK_WIDTH_METERS = 0.55;

    public final SwerveModule frontLeft = module(1, 2, 11);
    public final SwerveModule frontRight = module(3, 4, 12);
    public final SwerveModule backLeft = module(5, 6, 13);
    public final SwerveModule backRight = module(7, 8, 14);

    public Action drive(double forward, double strafe, double rotation) {
        SwerveModuleState[] states = kinematics().toSwerveModuleStates(new ChassisSpeeds(
                clamp(forward) * MAX_SPEED_METERS_PER_SECOND,
                clamp(strafe) * MAX_SPEED_METERS_PER_SECOND,
                clamp(rotation) * MAX_ROTATION_RADIANS_PER_SECOND));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECOND);

        return Actions.set()
                .set(frontLeft, frontLeft.target(target(states[0])))
                .set(frontRight, frontRight.target(target(states[1])))
                .set(backLeft, backLeft.target(target(states[2])))
                .set(backRight, backRight.target(target(states[3])));
    }

    private static SwerveModule module(int driveMotorId, int steerMotorId, int encoderId) {
        return new SwerveModules.SDS.MK5N.R3()
                .drive.fill(Constants.RIO.motor(MotorKinds.KRAKEN_X60, driveMotorId))
                .steer.fill(Constants.RIO.motor(MotorKinds.KRAKEN_X44, steerMotorId))
                .angle.fill(Constants.RIO.encoder(EncoderKinds.CANCODER, encoderId)
                        .units(EncoderUnit.ROTATIONS));
    }

    private static SwerveModuleTarget target(SwerveModuleState state) {
        return new SwerveModuleTarget(state.speedMetersPerSecond, state.angle.getRotations());
    }

    private static SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(WHEELBASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
                new Translation2d(WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
                new Translation2d(-WHEELBASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
                new Translation2d(-WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0));
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}

