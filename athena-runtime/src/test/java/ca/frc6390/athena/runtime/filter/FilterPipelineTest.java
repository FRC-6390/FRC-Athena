package ca.frc6390.athena.runtime.filter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

class FilterPipelineTest {
    @Test
    void filteredValueAppliesPipelineAndCachesLastOutput() {
        FilteredValue value = new FilteredValue(() -> 1.5).addOffset(0.25);

        double first = value.getFiltered();

        assertEquals(1.75, first, 1.0e-9);
        assertEquals(first, value.get(false), 1.0e-9);
    }

    @Test
    void filteredPoseAppliesAxisWiseFiltersAndCachesLastOutput() {
        AtomicReference<PoseSnapshot> input = new AtomicReference<>(new PoseSnapshot(2.0, -1.0, 0.5));
        FilteredPose pose = new FilteredPose(input::get).addMovingAverage(2);

        PoseSnapshot first = pose.getFiltered();
        assertEquals(first, pose.get(false));

        input.set(new PoseSnapshot(4.0, 1.0, 1.5));
        PoseSnapshot second = pose.getFiltered();

        assertEquals(second, pose.get(false));
        assertEquals(3.0, second.xMeters(), 1.0e-9);
        assertEquals(0.0, second.yMeters(), 1.0e-9);
        assertEquals(1.0, second.headingRadians(), 1.0e-9);
    }
}
