package ca.frc6390.athena.vision.device;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CameraClockStatusTest {
    @Test
    void freshPongAndAlignedFrameReportSynced() {
        CameraClockStatus status = CameraClockStatus.of(0.2, 10.0, 10.0);

        assertTrue(status.synced());
        assertEquals(0.2, status.timeSinceSync().in(Seconds), 1.0e-9);
        assertFalse(status.isUnknown());
    }

    @Test
    void pongOlderThanOneSecondIsUnsynced() {
        CameraClockStatus status = CameraClockStatus.of(1.5, 10.0, 10.0);

        assertFalse(status.synced());
        assertEquals(1.5, status.timeSinceSync().in(Seconds), 1.0e-9);
    }

    @Test
    void pongExactlyAtTimeoutIsUnsynced() {
        assertFalse(CameraClockStatus.of(CameraClockStatus.SYNC_TIMEOUT.in(Seconds), 10.0, 10.0).synced());
    }

    @Test
    void frameMoreThanTenthOfASecondAheadOfRobotTimeIsUnsynced() {
        CameraClockStatus status = CameraClockStatus.of(0.1, 10.25, 10.0);

        assertFalse(status.synced());
        assertEquals(0.1, status.timeSinceSync().in(Seconds), 1.0e-9);
    }

    @Test
    void frameWithinTenthOfASecondOfRobotTimeStillSynced() {
        assertTrue(CameraClockStatus.of(0.1, 10.05, 10.0).synced());
    }

    @Test
    void frameBehindRobotTimeIsNotFlagged() {
        assertTrue(CameraClockStatus.of(0.1, 9.0, 10.0).synced());
    }

    @Test
    void missingPongReportsUnknown() {
        CameraClockStatus status = CameraClockStatus.of(Double.NaN, 10.0, 10.0);

        assertFalse(status.synced());
        assertTrue(Double.isNaN(status.timeSinceSync().in(Seconds)));
        assertTrue(status.isUnknown());
        assertTrue(CameraClockStatus.of(-1.0, 10.0, 10.0).isUnknown());
    }

    @Test
    void nonFiniteRobotTimeDoesNotFlagFrame() {
        assertTrue(CameraClockStatus.of(0.1, 10.0, Double.NaN).synced());
    }

    @Test
    void unknownConstantAndDefaultCameraClockAreUnsynced() {
        assertEquals(CameraClockStatus.unknown(), CameraClockStatus.UNKNOWN);
        assertFalse(CameraClockStatus.UNKNOWN.synced());
        CameraClock clock = new CameraClock() { };
        assertTrue(clock.clockStatus().isUnknown());
    }

    @Test
    void recordNormalizesInvalidSecondsToNaN() {
        assertTrue(new CameraClockStatus(true, Seconds.of(-3.0)).isUnknown());
        assertTrue(new CameraClockStatus(true, Seconds.of(Double.POSITIVE_INFINITY)).isUnknown());
    }
}
