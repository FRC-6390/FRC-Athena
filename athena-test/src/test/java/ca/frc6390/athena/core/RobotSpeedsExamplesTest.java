package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.RobotSpeedsExamples;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotSpeedsExamplesTest {

    @AfterEach
    void resetRobotTimeCache() {
        RobotTime.resetNowSecondsForTest();
    }

    @Test
    void defaultProfileAddsAndClampsOutput() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(3.0, 2.0);
        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.5, 0.0, 1.5);
        speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 2.0, 0.0, 1.0);
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.5, 0.0, 0.7);

        ChassisSpeeds out = speeds.calculate();

        assertEquals(3.0, out.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, out.vyMetersPerSecond, 1e-9);
        assertEquals(2.0, out.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void translationAverageExampleProducesExpectedBlendedOutput() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(10.0, 10.0);
        RobotSpeedsExamples.configureDriveAutoAverageForTranslation(speeds);

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 4.0, 0.6);
        speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 2.0, 1.0);
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.25);

        ChassisSpeeds out = speeds.calculate();

        assertEquals(1.0, out.vxMetersPerSecond, 1e-9);
        assertEquals(3.0, out.vyMetersPerSecond, 1e-9);
        assertEquals(0.85, out.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void headingAssistOverrideExampleSupersedesTheta() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(10.0, 10.0);
        RobotSpeedsExamples.configureHeadingAssistOverride(speeds, "assist");

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.4);
        speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.2);
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.1);
        speeds.setSpeeds("assist", 0.0, 0.0, 1.3);

        ChassisSpeeds out = speeds.calculate();
        assertEquals(1.3, out.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void sourceBlendCycleIsRejected() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(10.0, 10.0);
        speeds.registerSpeedSource("assist");

        speeds.blend("drive", "auto", "feedback", RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.X);

        assertThrows(IllegalStateException.class,
                () -> speeds.blend("auto", "drive", "assist", RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.X));
    }

    @Test
    void fieldRelativeSourceUsesCurrentHeadingAtCalculationTime() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(10.0, 10.0);
        Rotation2d heading = Rotation2d.fromDegrees(90.0);

        speeds.setFieldRelativeSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 1.0);

        ChassisSpeeds out = speeds.calculate(heading);
        ChassisSpeeds expected = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 0.0, 1.0, heading);

        assertEquals(expected.vxMetersPerSecond, out.vxMetersPerSecond, 1e-9);
        assertEquals(expected.vyMetersPerSecond, out.vyMetersPerSecond, 1e-9);
        assertEquals(expected.omegaRadiansPerSecond, out.omegaRadiansPerSecond, 1e-9);
        assertTrue(speeds.isFieldRelativeSource(RobotSpeeds.DRIVE_SOURCE));
    }

    @Test
    void sourceUpdateTimestampTracksLatestWrite() {
        RobotSpeeds speeds = RobotSpeedsExamples.createDefaultProfile(10.0, 10.0);

        RobotTime.updateNowSeconds(1.25);
        speeds.setFieldRelativeSpeeds(RobotSpeeds.DRIVE_SOURCE, 1.0, 0.0, 0.0);
        assertEquals(1.25, speeds.getSourceLastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE), 1e-9);

        RobotTime.updateNowSeconds(2.0);
        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 1.0, 0.0);
        assertEquals(2.0, speeds.getSourceLastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE), 1e-9);
        assertFalse(speeds.isFieldRelativeSource(RobotSpeeds.DRIVE_SOURCE));
    }
}
