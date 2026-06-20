package ca.frc6390.athena.examples;

import ca.frc6390.athena.drivetrain.config.Drivetrains;
import ca.frc6390.athena.drivetrain.config.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;

/**
 * Swerve drivetrain example using generic module declarations.
 */
public final class SwerveExample {
    /**
     * Four-module swerve drivetrain declaration.
     */
    public static final SwerveDrivetrainConfig CONFIG = Drivetrains.swerve("swerve")
            .trackWidth(TrackWidth.meters(0.58))
            .wheelBase(WheelBase.meters(0.62))
            .module("frontLeft", module -> module
                    .location(0.31, 0.29)
                    .driveMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_LEFT_DRIVE).brake().currentLimit(45))
                    .steerMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_LEFT_STEER).brake().currentLimit(35))
                    .steerEncoder(encoder -> encoder.hardware(RobotHardware.SWERVE_FRONT_LEFT_ENCODER).absolutePosition())
                    .steerPid(4.5, 0.0, 0.12)
                    .driveFeedforward(0.18, 2.1, 0.3))
            .module("frontRight", module -> module
                    .location(0.31, -0.29)
                    .driveMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_RIGHT_DRIVE).brake().currentLimit(45))
                    .steerMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_RIGHT_STEER).brake().currentLimit(35))
                    .steerEncoder(encoder -> encoder.hardware(RobotHardware.SWERVE_FRONT_RIGHT_ENCODER).absolutePosition())
                    .steerPid(4.5, 0.0, 0.12)
                    .driveFeedforward(0.18, 2.1, 0.3))
            .module("backLeft", module -> module
                    .location(-0.31, 0.29)
                    .driveMotor(motor -> motor.hardware(RobotHardware.SWERVE_BACK_LEFT_DRIVE).brake().currentLimit(45))
                    .steerMotor(motor -> motor.hardware(RobotHardware.SWERVE_BACK_LEFT_STEER).brake().currentLimit(35))
                    .steerEncoder(encoder -> encoder.hardware(RobotHardware.SWERVE_BACK_LEFT_ENCODER).absolutePosition())
                    .steerPid(4.5, 0.0, 0.12)
                    .driveFeedforward(0.18, 2.1, 0.3))
            .module("backRight", module -> module
                    .location(-0.31, -0.29)
                    .driveMotor(motor -> motor.hardware(RobotHardware.SWERVE_BACK_RIGHT_DRIVE).brake().currentLimit(45))
                    .steerMotor(motor -> motor.hardware(RobotHardware.SWERVE_BACK_RIGHT_STEER).brake().currentLimit(35))
                    .steerEncoder(encoder -> encoder.hardware(RobotHardware.SWERVE_BACK_RIGHT_ENCODER).absolutePosition())
                    .steerPid(4.5, 0.0, 0.12)
                    .driveFeedforward(0.18, 2.1, 0.3));

    private SwerveExample() {
    }
}
