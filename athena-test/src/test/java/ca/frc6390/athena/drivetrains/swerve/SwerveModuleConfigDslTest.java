package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

final class SwerveModuleConfigDslTest {
    private static final MotorControllerType DUMMY_MOTOR = () -> "dummy:motor";
    private static final EncoderType DUMMY_ENCODER = () -> "dummy:encoder";

    private static SwerveModule.SwerveModuleConfig baseConfig() {
        MotorControllerConfig driveMotor = MotorControllerConfig.create(DUMMY_MOTOR, 1)
                .encoder(e -> e.config(EncoderConfig.create(DUMMY_ENCODER, 11)));
        MotorControllerConfig steerMotor = MotorControllerConfig.create(DUMMY_MOTOR, 2)
                .encoder(e -> e.config(EncoderConfig.create(DUMMY_ENCODER, 12)));
        return new SwerveModule.SwerveModuleConfig(
                new Translation2d(),
                0.1016,
                4.5,
                driveMotor,
                steerMotor,
                new PIDController(0.0, 0.0, 0.0),
                EncoderConfig.create(DUMMY_ENCODER, 12),
                SwerveModule.SwerveModuleSimConfig.fromMotors(null, null));
    }

    @Test
    void pidConsumerAppliesPerModuleGains() {
        SwerveModule.SwerveModuleConfig cfg = baseConfig()
                .pid(p -> p.p(0.30).i(0.02).d(0.001));

        assertNotNull(cfg.rotationPID());
        assertEquals(0.30, cfg.rotationPID().getP(), 1e-9);
        assertEquals(0.02, cfg.rotationPID().getI(), 1e-9);
        assertEquals(0.001, cfg.rotationPID().getD(), 1e-9);
    }

    @Test
    void ffConsumerAppliesPerModuleDriveFeedforward() {
        SwerveModule.SwerveModuleConfig cfg = baseConfig()
                .ff(f -> f.ks(0.16).kv(2.35).ka(0.12));

        assertNotNull(cfg.driveFeedforward());
        assertTrue(cfg.driveFeedforwardEnabled());
        assertEquals(0.16, cfg.driveFeedforward().getKs(), 1e-9);
        assertEquals(2.35, cfg.driveFeedforward().getKv(), 1e-9);
        assertEquals(0.12, cfg.driveFeedforward().getKa(), 1e-9);
    }

    @Test
    void ffEnabledCanDisableConfiguredFeedforwardPerModule() {
        SwerveModule.SwerveModuleConfig cfg = baseConfig()
                .ff(f -> f.ks(0.10).kv(2.10).ka(0.09))
                .ffEnabled(false);

        assertNotNull(cfg.driveFeedforward());
        assertFalse(cfg.driveFeedforwardEnabled());
    }

    @Test
    void inversionDslTracksExplicitOverrides() {
        SwerveModule.SwerveModuleConfig cfg = baseConfig()
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(true);

        assertTrue(cfg.driveInvertedExplicit());
        assertTrue(cfg.steerInvertedExplicit());
        assertTrue(cfg.encoderInvertedExplicit());
    }
}
