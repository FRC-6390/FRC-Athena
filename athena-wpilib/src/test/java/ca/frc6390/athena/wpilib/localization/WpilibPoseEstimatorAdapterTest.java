package ca.frc6390.athena.wpilib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.junit.jupiter.api.Test;

class WpilibPoseEstimatorAdapterTest {
    @Test
    void standardDeviationsMapAthenaValuesToWpilibVector() {
        WpilibPoseEstimatorAdapter adapter = new WpilibPoseEstimatorAdapter((pose, timestamp, standardDeviations) -> {
        });

        Matrix<N3, N1> vector = adapter.standardDeviations(MeasurementStdDevs.of(0.4, 0.5, 0.6));

        assertEquals(0.4, vector.get(0, 0), 1.0e-9);
        assertEquals(0.5, vector.get(1, 0), 1.0e-9);
        assertEquals(0.6, vector.get(2, 0), 1.0e-9);
    }

    @Test
    void nullStandardDeviationsDefaultToInfinity() {
        WpilibPoseEstimatorAdapter adapter = new WpilibPoseEstimatorAdapter((pose, timestamp, standardDeviations) -> {
        });

        Matrix<N3, N1> vector = adapter.standardDeviations(null);

        assertEquals(Double.POSITIVE_INFINITY, vector.get(0, 0));
        assertEquals(Double.POSITIVE_INFINITY, vector.get(1, 0));
        assertEquals(Double.POSITIVE_INFINITY, vector.get(2, 0));
    }

    @Test
    void addVisionMeasurementSanitizesTimestampAndForwardsPoseAndDeviations() {
        RecordingSink sink = new RecordingSink();
        WpilibPoseEstimatorAdapter adapter = new WpilibPoseEstimatorAdapter(sink);
        Pose2d pose = new Pose2d(1.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.fromRadians(0.3));

        adapter.addVisionMeasurement(pose, Double.NaN, MeasurementStdDevs.of(0.1, 0.2, 0.3));

        assertSame(pose, sink.pose);
        assertEquals(0.0, sink.timestampSeconds, 1.0e-9);
        assertEquals(0.1, sink.standardDeviations.get(0, 0), 1.0e-9);
        assertEquals(0.2, sink.standardDeviations.get(1, 0), 1.0e-9);
        assertEquals(0.3, sink.standardDeviations.get(2, 0), 1.0e-9);
    }

    private static final class RecordingSink implements WpilibPoseEstimatorAdapter.VisionMeasurementSink {
        private Pose2d pose;
        private double timestampSeconds = Double.NaN;
        private Matrix<N3, N1> standardDeviations;

        @Override
        public void addVisionMeasurement(
                Pose2d pose,
                double timestampSeconds,
                Matrix<N3, N1> standardDeviations) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.standardDeviations = standardDeviations;
        }
    }
}
