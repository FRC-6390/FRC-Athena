package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class LoopTimingAndRobotTimeTest {

    @AfterEach
    void resetLoopTimingState() {
        LoopTiming.setDebugAlways(false);
        LoopTiming.clearMechanismDurationsForTest();
        RobotTime.resetNowSecondsForTest();
    }

    @Test
    void beginCycleCachesRobotTimeAndSamplingFlag() {
        LoopTiming.setDebugAlways(true);
        LoopTiming.beginCycle();

        assertTrue(Double.isFinite(RobotTime.nowSeconds()));
        assertTrue(LoopTiming.shouldSampleMechanisms());
    }

    @Test
    void recordMechanismIgnoredWhenSamplingDisabled() {
        LoopTiming.setDebugAlways(false);
        LoopTiming.beginCycle();

        LoopTiming.recordMechanism("arm", 3.0);
        assertTrue(LoopTiming.mechanismDurationsForTest().isEmpty());
    }

    @Test
    void recordMechanismAccumulatesWhenSamplingEnabled() {
        LoopTiming.setDebugAlways(true);
        LoopTiming.beginCycle();

        LoopTiming.recordMechanism("arm", 3.0);
        LoopTiming.recordMechanism("arm", 2.5);
        LoopTiming.recordMechanism("elevator", 1.0);

        Map<String, Double> map = LoopTiming.mechanismDurationsForTest();
        assertEquals(5.5, map.get("arm"), 1e-9);
        assertEquals(1.0, map.get("elevator"), 1e-9);
    }

    @Test
    void robotTimeUpdateNowSecondsIsDirectlyReadable() {
        RobotTime.updateNowSeconds(123.456);
        assertEquals(123.456, RobotTime.nowSeconds(), 1e-9);
    }

    @Test
    void robotTimeSnapshotTracksLoopDtAndCycleCount() {
        RobotTime.updateNowSeconds(10.0);
        RobotTime.updateNowSeconds(10.02);

        assertEquals(10.02, RobotTime.nowSeconds(), 1e-9);
        assertEquals(0.02, RobotTime.loopDtSeconds(), 1e-9);
        assertEquals(2L, RobotTime.loopCycleCount());

        RobotTime.LoopSnapshot snapshot = RobotTime.loopSnapshot();
        assertEquals(10.02, snapshot.nowSeconds(), 1e-9);
        assertEquals(0.02, snapshot.loopDtSeconds(), 1e-9);
        assertEquals(2L, snapshot.cycleCount());
        assertTrue(snapshot.hasNowSeconds());
        assertTrue(snapshot.hasLoopDtSeconds());
    }

    @Test
    void robotTimeProjectedNowAdvancesMonotonicallyFromCachedSample() {
        RobotTime.updateNowSeconds(42.0);

        double projectedNow = RobotTime.nowSecondsProjected();
        assertTrue(projectedNow >= 42.0);
        assertTrue(projectedNow <= 42.25);
    }

    @Test
    void robotTimeResetClearsSnapshot() {
        RobotTime.updateNowSeconds(5.0);
        RobotTime.updateNowSeconds(5.02);

        RobotTime.resetNowSecondsForTest();

        assertTrue(Double.isNaN(RobotTime.nowSeconds()));
        assertTrue(Double.isNaN(RobotTime.loopDtSeconds()));
        assertEquals(0L, RobotTime.loopCycleCount());
    }
}
