package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.testsupport.hardware.TestSimHardwareFactory.TestEncoderType;
import ca.frc6390.athena.testsupport.hardware.TestSimHardwareFactory.TestMotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

final class RobotCoreConfigAutoSectionTest {

    @Test
    void autoConfigFollowerPeriodDefaultsToFiveMilliseconds() {
        assertEquals(0.005, RobotCore.AutoConfig.defaults().followerPeriodSeconds(), 1e-9);
    }

    @Test
    void autoSectionSupportsAxisDslAndInvertedFlags() {
        RobotCore.RobotCoreConfig<SwerveDrivetrain> cfg = RobotCoreConfig.create()
                .drivetrain(d -> d.swerve(SwerveDrivetrainConfig.standard(0.6)))
                .auto(a -> a
                        .translation(p -> p
                                .kp(7.0)
                                .ki(0.2)
                                .kd(0.1)
                                .iZone(1.5)
                                .inverted(true))
                                .rotation(p -> p
                                .kp(2.0)
                                .ki(0.0)
                                .kd(0.05)
                                .inverted(false))
                        .followerPeriodMs(20.0))
                .build();

        HolonomicPidConstants translation = cfg.autoConfig().translationPid();
        HolonomicPidConstants rotation = cfg.autoConfig().rotationPid();

        assertEquals(7.0, translation.kP(), 1e-9);
        assertEquals(0.2, translation.kI(), 1e-9);
        assertEquals(0.1, translation.kD(), 1e-9);
        assertEquals(1.5, translation.iZone(), 1e-9);
        assertTrue(translation.inverted());

        assertEquals(2.0, rotation.kP(), 1e-9);
        assertEquals(0.0, rotation.kI(), 1e-9);
        assertEquals(0.05, rotation.kD(), 1e-9);
        assertFalse(rotation.inverted());
        assertEquals(0.02, cfg.autoConfig().followerPeriodSeconds(), 1e-9);
    }

    @Test
    void nestedRobotCoreConfigSupportsAutoAxisDslAndInvertedFlags() {
        RobotCore.RobotCoreConfig<SwerveDrivetrain> cfg = RobotCore.RobotCoreConfig
                .swerve(SwerveDrivetrainConfig.standard(0.6))
                .auto(a -> a
                        .translation(p -> p.kp(4.0).inverted(false))
                        .rotation(p -> p.kp(1.5).inverted(true))
                        .followerPeriodSeconds(0.01));

        assertEquals(4.0, cfg.autoConfig().translationPid().kP(), 1e-9);
        assertFalse(cfg.autoConfig().translationPid().inverted());
        assertEquals(1.5, cfg.autoConfig().rotationPid().kP(), 1e-9);
        assertTrue(cfg.autoConfig().rotationPid().inverted());
        assertEquals(0.01, cfg.autoConfig().followerPeriodSeconds(), 1e-9);
    }

    @Test
    void drivetrainSectionTimingKnobsApplyToBuiltSwerveDrivetrain() {
        RobotCore.RobotCoreConfig<SwerveDrivetrain> cfg = RobotCoreConfig.create()
                .drivetrain(d -> d
                        .updatePeriodMs(20.0)
                        .driverCommandPeriodMs(10.0)
                        .swerve(configuredSwerveConfig()))
                .build();

        SwerveDrivetrain drivetrain = cfg.driveTrain().build();
        assertEquals(0.02, drivetrain.updatePeriodSeconds(), 1e-9);
        assertEquals(0.01, drivetrain.driverCommandPeriodSeconds(), 1e-9);
    }

    private static SwerveDrivetrainConfig configuredSwerveConfig() {
        double trackWidthMeters = 0.57;
        double wheelbaseMeters = 0.57;
        Translation2d[] locations = SwerveModule.SwerveModuleConfig.generateModuleLocations(
                trackWidthMeters,
                wheelbaseMeters);
        return SwerveDrivetrainConfig.create()
                .hardware(h -> h
                        .moduleLocations(locations)
                        .modules(
                                createModuleConfig(locations[0], 1, 5, 9),
                                createModuleConfig(locations[1], 2, 6, 10),
                                createModuleConfig(locations[2], 3, 7, 11),
                                createModuleConfig(locations[3], 4, 8, 12)));
    }

    private static SwerveModule.SwerveModuleConfig createModuleConfig(
            Translation2d location,
            int driveId,
            int steerId,
            int encoderId) {
        double wheelDiameterMeters = Units.inchesToMeters(4.0);
        double driveRatio = 1.0 / 6.12;
        double steerRatio = 150.0 / 7.0;
        double neoFreeSpeedRpm = 5820.0;
        double maxSpeedMetersPerSecond = (neoFreeSpeedRpm / 60.0) * driveRatio * Math.PI * wheelDiameterMeters;

        MotorControllerConfig driveMotor = MotorControllerConfig.create(TestMotorType.INSTANCE, driveId)
                .encoder(e -> e.config(EncoderConfig.create(TestEncoderType.INSTANCE, driveId)
                        .measurement(m -> m
                                .gearRatio(driveRatio)
                                .conversion(Math.PI * wheelDiameterMeters))));

        MotorControllerConfig steerMotor = MotorControllerConfig.create(TestMotorType.INSTANCE, steerId)
                .encoder(e -> e.config(EncoderConfig.create(TestEncoderType.INSTANCE, steerId)
                        .measurement(m -> m.gearRatio(steerRatio))));

        EncoderConfig moduleEncoder = EncoderConfig.create(TestEncoderType.INSTANCE, encoderId)
                .measurement(m -> m.gearRatio(1.0));

        return new SwerveModule.SwerveModuleConfig(
                location,
                wheelDiameterMeters,
                maxSpeedMetersPerSecond,
                driveMotor,
                steerMotor,
                new PIDController(40.0, 0.0, 0.0),
                moduleEncoder,
                SwerveModule.SwerveModuleSimConfig.fromMotors(AthenaMotor.NEO_V1, AthenaMotor.NEO_V1));
    }
}
