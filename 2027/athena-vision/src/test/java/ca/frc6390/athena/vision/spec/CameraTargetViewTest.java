package ca.frc6390.athena.vision.spec;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CameraTargetViewTest {
    @Test
    void returnsEmptyValuesWhenNoValidTargetExists() {
        CameraTargetView view = new CameraTargetView(VisionFrame.of(
                VisionObservation.tag(3, Double.NaN, 1.0, 2.0, 0.8)));

        assertFalse(view.hasValidTarget());
        assertTrue(view.yawDegrees().isEmpty());
        assertTrue(view.tagId().isEmpty());
        assertTrue(view.observations().isEmpty());
    }

    @Test
    void exposesBestTargetValuesWhenFrameIsValid() {
        CameraTargetView view = new CameraTargetView(VisionFrame.of(
                VisionObservation.tag(3, 8.5, -2.0, 4.1, 0.7),
                VisionObservation.tag(7, -3.2, -1.5, 2.4, 0.92)));

        assertTrue(view.hasValidTarget());
        assertEquals(7, view.tagId().orElseThrow());
        assertEquals(-3.2, view.yawDegrees().orElseThrow());
        assertEquals(2, view.observations().size());
    }
}
