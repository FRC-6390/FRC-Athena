package ca.frc6390.athena.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.vision.runtime.VisionTurnAssist;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;

class VisionFrameTest {
    @Test
    void emptyFrameHasNoTarget() {
        VisionFrame frame = VisionFrame.noTarget();

        assertFalse(frame.hasValidTarget());
        assertTrue(frame.yawDegrees().isEmpty());
        assertTrue(frame.tagId().isEmpty());
    }

    @Test
    void nonFiniteYawIsNoTarget() {
        VisionFrame frame = VisionFrame.of(VisionObservation.tag(7, Double.NaN, 2.0, 3.0, 1.0));

        assertFalse(frame.hasValidTarget());
        assertTrue(frame.validObservations().isEmpty());
        assertTrue(frame.yawDegrees().isEmpty());
    }

    @Test
    void selectsHigherConfidenceTargetFirst() {
        VisionObservation lower = VisionObservation.tag(1, 12.0, 0.5, 1.0, 0.5);
        VisionObservation higher = VisionObservation.tag(2, -4.0, 0.2, 5.0, 0.8);

        VisionFrame frame = VisionFrame.of(lower, higher);

        assertTrue(frame.hasValidTarget());
        assertEquals(2, frame.tagId().orElseThrow());
        assertEquals(-4.0, frame.yawDegrees().orElseThrow());
    }

    @Test
    void tiesPreferCloserDistanceThenTranslation() {
        VisionObservation far = new VisionObservation(1, 8.0, 0.0, 5.0, 0.0, 0.0, 0.8);
        VisionObservation closeWide = new VisionObservation(2, 6.0, 0.0, 2.0, 3.0, 0.0, 0.8);
        VisionObservation closeNear = new VisionObservation(3, 4.0, 0.0, 2.0, 1.0, 0.0, 0.8);

        VisionFrame frame = VisionFrame.of(far, closeWide, closeNear);

        assertEquals(3, frame.tagId().orElseThrow());
        assertEquals(2.0, frame.distanceMeters().orElseThrow());
        assertEquals(4.0, frame.yawDegrees().orElseThrow());
    }

    @Test
    void negativeDistanceIsInvalid() {
        VisionFrame frame = VisionFrame.of(VisionObservation.tag(1, 1.0, 1.0, -1.0, 1.0));

        assertFalse(frame.hasValidTarget());
        assertTrue(frame.distanceMeters().isEmpty());
    }

    @Test
    void visionTurnAssistWritesRobotSpeedFeedback() {
        RobotSpeeds speeds = new RobotSpeeds(4.5, 3.0);
        VisionTurnAssist assist = new VisionTurnAssist(
                speeds,
                () -> VisionFrame.of(VisionObservation.tag(3, 10.0, 0.0, 2.0, 0.9)),
                -2.0,
                0.05);

        assist.execute();

        assertEquals(Math.toRadians(10.0) * -2.0, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
        assertFalse(assist.isAligned());

        assist.stop();

        assertEquals(0.0, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }
}
