package ca.frc6390.athena.filters;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import ca.frc6390.athena.filters.examples.FilterExamples;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class FilterExamplesTest {

    @Test
    void offsetValueAddsConfiguredOffsetAndCachesLastOutput() {
        FilteredValue filtered = FilterExamples.offsetValue(() -> 1.5, 0.25);

        double first = filtered.getFiltered();
        double cached = filtered.get(false);

        assertEquals(1.75, first, 1e-9);
        assertEquals(first, cached, 1e-9);
    }

    @Test
    void smoothedPoseProducesPoseOutput() {
        Pose2d input = new Pose2d(2.0, -1.0, Rotation2d.fromDegrees(30.0));
        FilteredPose filtered = FilterExamples.smoothedPose(() -> input);

        Pose2d result = filtered.getFiltered();

        assertNotNull(result);
        assertEquals(result, filtered.get(false));
    }
}
