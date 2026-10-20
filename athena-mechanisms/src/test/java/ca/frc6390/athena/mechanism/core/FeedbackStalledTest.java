package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FeedbackStalledTest {
    @Test
    void stalledAwayFromTheGoalReportsTheCurrent() {
        assertEquals(30.0, ControlBinding.stallMetric(6.0, 0.0, -30.0, 40.0, 2.0), 1e-9);
    }

    @Test
    void holdingStillAtTheGoalIsNotAStall() {
        assertEquals(0.0, ControlBinding.stallMetric(1.5, 0.0, 12.0, 0.5, 2.0), 1e-9);
        assertEquals(0.0, ControlBinding.stallMetric(1.5, 0.0, 12.0, -2.0, 2.0), 1e-9);
    }

    @Test
    void movingOrUndrivenIsNotAStall() {
        assertEquals(0.0, ControlBinding.stallMetric(6.0, 5.0, 30.0, 40.0, 2.0), 1e-9);
        assertEquals(0.0, ControlBinding.stallMetric(0.2, 0.0, 30.0, 40.0, 2.0), 1e-9);
    }

    @Test
    void withoutASampleTheStallStillCounts() {
        assertEquals(30.0, ControlBinding.stallMetric(6.0, 0.0, 30.0, Double.NaN, 2.0), 1e-9);
    }
}
