package ca.frc6390.athena.runtime.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotSpeedsTest {
    @Test
    void defaultProfileAddsAndClampsOutput() {
        RobotSpeeds speeds = new RobotSpeeds(3.0, 2.0)
                .setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.5, 0.0, 1.5)
                .setSpeeds(RobotSpeeds.AUTO_SOURCE, 2.0, 0.0, 1.0)
                .setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.5, 0.0, 0.7);

        RobotVelocity out = speeds.calculate();

        assertEquals(3.0, out.xMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, out.yMetersPerSecond(), 1.0e-9);
        assertEquals(2.0, out.angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void translationAverageProducesExpectedBlendedOutput() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0)
                .clearBlends()
                .blend(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.AUTO_SOURCE,
                        RobotSpeeds.BlendMode.AVERAGE, RobotSpeeds.SpeedAxis.X, RobotSpeeds.SpeedAxis.Y)
                .blendToOutput(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.ALL)
                .blendToOutput(RobotSpeeds.FEEDBACK_SOURCE, RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.THETA)
                .setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 4.0, 0.6)
                .setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 2.0, 1.0)
                .setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.25);

        RobotVelocity out = speeds.calculate();

        assertEquals(1.0, out.xMetersPerSecond(), 1.0e-9);
        assertEquals(3.0, out.yMetersPerSecond(), 1.0e-9);
        assertEquals(0.85, out.angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void headingAssistCanSupersedeThetaOutput() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0)
                .registerSource("assist")
                .blendToOutput("assist", RobotSpeeds.BlendMode.B_SUPERSEDES_A, RobotSpeeds.SpeedAxis.THETA)
                .setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.4)
                .setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.2)
                .setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.1)
                .setSpeeds("assist", 0.0, 0.0, 1.3);

        RobotVelocity out = speeds.calculate();

        assertEquals(1.3, out.angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void sourceBlendCycleIsRejected() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0)
                .registerSource("assist")
                .blend("drive", "auto", "feedback", RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.X);

        assertThrows(IllegalStateException.class,
                () -> speeds.blend("auto", "drive", "assist", RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.X));
    }

    @Test
    void fieldRelativeSourceUsesCurrentHeadingAtCalculationTime() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0)
                .setFieldRelativeSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 1.0);

        RobotVelocity out = speeds.calculate(Math.PI / 2.0);

        assertEquals(0.0, out.xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, out.yMetersPerSecond(), 1.0e-9);
        assertEquals(1.0, out.angularRadiansPerSecond(), 1.0e-9);
        assertTrue(speeds.isFieldRelative(RobotSpeeds.DRIVE_SOURCE));
    }

    @Test
    void sourceUpdateTimestampTracksLatestWrite() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);

        speeds.nowSeconds(1.25).setFieldRelativeSpeeds(RobotSpeeds.DRIVE_SOURCE, 1.0, 0.0, 0.0);
        assertEquals(1.25, speeds.lastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE), 1.0e-9);

        speeds.nowSeconds(2.0).setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 1.0, 0.0);
        assertEquals(2.0, speeds.lastUpdateSeconds(RobotSpeeds.DRIVE_SOURCE), 1.0e-9);
        assertFalse(speeds.isFieldRelative(RobotSpeeds.DRIVE_SOURCE));
    }
}
