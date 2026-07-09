package ca.frc6390.athena.localization.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

class LocalizationPipelineTest {
    @Test
    void inputChainingUsesNestedPipelineResults() {
        LocalizationPipeline odometry = Localizations.odometry()
                .input(signal(sample(1.0, 2.0, 0.0, 5.0, 1.0)));
        LocalizationPipeline fused = Localizations.latestValid().input(odometry);

        assertPose(fused.pose(), 1.0, 2.0, 0.0);
        assertEquals(1, fused.measurements().size());
    }

    @Test
    void fieldBoundsRejectsOutOfBoundsMeasurementsAndKeepsLatestAcceptedPose() {
        MutableSignal input = new MutableSignal(sample(2.0, 2.0, 0.0, 1.0, 1.0));
        LocalizationPipeline pipeline = Localizations.latestValid()
                .input(input)
                .filter(new FieldBoundsFilter(0.0, 0.0, 5.0, 5.0));

        assertPose(pipeline.pose(), 2.0, 2.0, 0.0);

        input.measurements = List.of(sample(8.0, 8.0, 0.0, 2.0, 1.0));

        assertPose(pipeline.pose(), 2.0, 2.0, 0.0);
    }

    @Test
    void weightedAveragePrefersLowerTranslationVarianceAndUsesLatestTimestamp() {
        Measurement weak = sample(0.0, 0.0, 0.0, 4.0, 2.0);
        Measurement strong = sample(10.0, 0.0, 0.0, 6.0, 1.0);
        LocalizationPipeline pipeline = Localizations.weightedAverage()
                .input(signal(weak, strong));

        assertPose(pipeline.pose(), 8.0, 0.0, 0.0);
        assertEquals(1, pipeline.measurements().size());
    }

    @Test
    void latestValidChoosesNewestPoseMeasurement() {
        LocalizationPipeline pipeline = Localizations.latestValid()
                .input(signal(
                        sample(1.0, 0.0, 0.0, 1.0, 1.0),
                        sample(3.0, 0.0, 0.0, 3.0, 1.0),
                        sample(2.0, 0.0, 0.0, 2.0, 1.0)));

        assertPose(pipeline.pose(), 3.0, 0.0, 0.0);
    }

    @Test
    void resetAndResetToUpdateSharedPipelineState() {
        LocalizationPipeline source = Localizations.odometry();
        LocalizationPipeline copy = Localizations.vision();

        source.reset(4.0, 5.0, 0.25).apply(null);
        copy.resetTo(source).apply(null);

        assertPose(source.pose(), 4.0, 5.0, 0.25);
        assertPose(copy.pose(), 4.0, 5.0, 0.25);
    }

    @Test
    void noPoseInputsProduceEmptyMeasurementsAndZeroDefaultPose() {
        Measurement custom = new Measurement() {
            @Override
            public double timestampSeconds() {
                return 1.0;
            }

            @Override
            public double latencySeconds() {
                return 0.0;
            }

            @Override
            public Object source() {
                return this;
            }
        };
        LocalizationPipeline pipeline = Localizations.latestValid().input(signal(custom));

        assertTrue(pipeline.measurements().isEmpty());
        assertPose(pipeline.pose(), 0.0, 0.0, 0.0);
    }

    private static MeasurementSignal signal(Measurement... measurements) {
        return () -> List.of(measurements);
    }

    private static PoseSampleMeasurement sample(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            double translationStdDev) {
        return new PoseSampleMeasurement(
                new PoseSnapshot(xMeters, yMeters, headingRadians),
                RobotVelocity.zero(),
                timestampSeconds,
                MeasurementStdDevs.of(translationStdDev, translationStdDev, 1.0));
    }

    private static void assertPose(PoseSnapshot pose, double xMeters, double yMeters, double headingRadians) {
        assertEquals(xMeters, pose.xMeters(), 1.0e-9);
        assertEquals(yMeters, pose.yMeters(), 1.0e-9);
        assertEquals(headingRadians, pose.headingRadians(), 1.0e-9);
    }

    private static final class MutableSignal implements MeasurementSignal {
        private List<Measurement> measurements;

        private MutableSignal(Measurement measurement) {
            measurements = List.of(measurement);
        }

        @Override
        public List<Measurement> measurements() {
            return measurements;
        }
    }

    private record PoseSampleMeasurement(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            MeasurementStdDevs stdDevs) implements PoseMeasurementSample {
        @Override
        public double latencySeconds() {
            return 0.0;
        }

        @Override
        public double ambiguity() {
            return 0.0;
        }

        @Override
        public int targetCount() {
            return 1;
        }

        @Override
        public Object source() {
            return this;
        }
    }
}
