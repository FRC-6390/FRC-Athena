package ca.frc6390.athena.drivetrains.swerve.examples;

import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Example Swerve module configuration snippets.
 */
public final class SwerveDrivetrainExamples {
    private static final MotorControllerType DUMMY_MOTOR = () -> "dummy:motor";
    private static final EncoderType DUMMY_ENCODER = () -> "dummy:encoder";

    private SwerveDrivetrainExamples() {}

    public static SwerveModule.SwerveModuleConfig moduleWithPidAndFeedforward() {
        MotorControllerConfig driveMotor = MotorControllerConfig.create(DUMMY_MOTOR, 1)
                .encoder(e -> e.config(EncoderConfig.create(DUMMY_ENCODER, 11)));
        MotorControllerConfig steerMotor = MotorControllerConfig.create(DUMMY_MOTOR, 2)
                .encoder(e -> e.config(EncoderConfig.create(DUMMY_ENCODER, 12)));

        return new SwerveModule.SwerveModuleConfig(
                new Translation2d(0.25, 0.25),
                0.1016,
                4.5,
                driveMotor,
                steerMotor,
                new PIDController(0.0, 0.0, 0.0),
                EncoderConfig.create(DUMMY_ENCODER, 12))
                .pid(p -> p.p(0.3).i(0.02).d(0.001))
                .ff(f -> f.ks(0.16).kv(2.35).ka(0.12))
                .driveInverted(true)
                .steerInverted(false)
                .encoderInverted(true);
    }
}
