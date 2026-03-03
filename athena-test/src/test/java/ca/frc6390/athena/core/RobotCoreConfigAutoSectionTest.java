package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import org.junit.jupiter.api.Test;

final class RobotCoreConfigAutoSectionTest {

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
                                .inverted(false)))
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
    }

    @Test
    void nestedRobotCoreConfigSupportsAutoAxisDslAndInvertedFlags() {
        RobotCore.RobotCoreConfig<SwerveDrivetrain> cfg = RobotCore.RobotCoreConfig
                .swerve(SwerveDrivetrainConfig.standard(0.6))
                .auto(a -> a
                        .translation(p -> p.kp(4.0).inverted(false))
                        .rotation(p -> p.kp(1.5).inverted(true)));

        assertEquals(4.0, cfg.autoConfig().translationPid().kP(), 1e-9);
        assertFalse(cfg.autoConfig().translationPid().inverted());
        assertEquals(1.5, cfg.autoConfig().rotationPid().kP(), 1e-9);
        assertTrue(cfg.autoConfig().rotationPid().inverted());
    }
}
