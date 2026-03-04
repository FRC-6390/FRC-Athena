package ca.frc6390.athena.drivetrains.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

final class SwerveDrivetrainConfigInversionTest {

    @Test
    void resolvesInversionFromGlobalFlagAndSignedIds() {
        assertEquals(7, SwerveDrivetrainConfig.resolvedModuleId(7));
        assertEquals(7, SwerveDrivetrainConfig.resolvedModuleId(-7));

        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(false, 4));
        assertTrue(SwerveDrivetrainConfig.resolvedModuleInverted(false, -4));
        assertTrue(SwerveDrivetrainConfig.resolvedModuleInverted(true, 4));
        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(true, -4));
    }

    @Test
    void moduleOverrideWinsWhenDrivetrainOverrideNotConfigured() {
        assertTrue(SwerveDrivetrainConfig.resolvedModuleInverted(
                true,
                true,
                true,
                false,
                false,
                4));

        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(
                true,
                true,
                false,
                false,
                true,
                4));
    }

    @Test
    void drivetrainOverrideWinsWhenConfigured() {
        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(
                true,
                true,
                true,
                true,
                false,
                4));

        assertTrue(SwerveDrivetrainConfig.resolvedModuleInverted(
                false,
                true,
                false,
                true,
                true,
                4));
    }

    @Test
    void signedIdsStillFlipResolvedBaseInversion() {
        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(
                false,
                true,
                true,
                false,
                false,
                -4));

        assertFalse(SwerveDrivetrainConfig.resolvedModuleInverted(
                true,
                false,
                false,
                false,
                false,
                -4));
    }

    @Test
    void clonedModuleConfigsDoNotShareMutableMotorOrEncoderConfigs() {
        MotorControllerConfig drive = new MotorControllerConfig();
        MotorControllerConfig steer = new MotorControllerConfig();
        EncoderConfig encoder = EncoderConfig.create();

        SwerveModule.SwerveModuleConfig base = new SwerveModule.SwerveModuleConfig(
                new Translation2d(0.2, -0.2),
                0.1016,
                4.5,
                drive,
                steer,
                new PIDController(1.0, 0.0, 0.0),
                encoder);
        SwerveModule.SwerveModuleConfig clone = new SwerveModule.SwerveModuleConfig(base);

        assertNotSame(base.driveMotor(), clone.driveMotor());
        assertNotSame(base.rotationMotor(), clone.rotationMotor());
        assertNotSame(base.encoder(), clone.encoder());

        clone.driveId(11).steerId(12).encoderId(13).driveInverted(true).steerInverted(true).encoderInverted(true);

        assertEquals(0, base.driveMotor().id());
        assertEquals(0, base.rotationMotor().id());
        assertEquals(0, base.encoder().id());
        assertFalse(base.driveMotor().inverted());
        assertFalse(base.rotationMotor().inverted());
        assertFalse(base.encoder().inverted());
    }

    @Test
    void updatePeriodDefaultsToFiveMilliseconds() throws Exception {
        SwerveDrivetrainConfig config = SwerveDrivetrainConfig.create();
        assertEquals(0.005, readUpdatePeriodSeconds(config), 1e-9);
    }

    @Test
    void updatePeriodCanBeConfiguredThroughControlAndConfigSections() throws Exception {
        SwerveDrivetrainConfig config = SwerveDrivetrainConfig.create();

        config.control(c -> c.updatePeriodMs(7.5));
        assertEquals(0.0075, readUpdatePeriodSeconds(config), 1e-9);

        config.config(c -> c.updatePeriodSeconds(0.012));
        assertEquals(0.012, readUpdatePeriodSeconds(config), 1e-9);

        // Invalid values are ignored and preserve the last valid period.
        config.control(c -> c.updatePeriodSeconds(-1.0));
        assertEquals(0.012, readUpdatePeriodSeconds(config), 1e-9);
    }

    private static double readUpdatePeriodSeconds(SwerveDrivetrainConfig config) throws Exception {
        Field field = SwerveDrivetrainConfig.class.getDeclaredField("updatePeriodSeconds");
        field.setAccessible(true);
        return field.getDouble(config);
    }
}
