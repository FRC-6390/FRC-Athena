package ca.frc6390.athena.localization.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import java.util.List;
import org.junit.jupiter.api.Test;

class LocalizationTest {
    private static final double EPSILON = 1.0e-9;

    @Test
    void filterPreservesAcceptedMeasurementMetadataAndTracksRejections() {
        Object camera = new Object();
        Sample accepted = sample(2.0, 3.0, 0.25, 4.0, 0.08, 3, 2.5,
                MeasurementStdDevs.of(0.2, 0.3, 0.1), camera);
        Sample rejected = sample(9.0, 9.0, 0.5, 5.0, 0.35, 1, 7.0,
                MeasurementStdDevs.of(1.0, 1.1, 0.8), camera);

        Localization stage = Localizations.filter()
                .input(signal(accepted, rejected))
                .filter((localization, measurement, pose) -> pose.xMeters() < 5.0);

        assertEquals(List.of(accepted), stage.measurements());
        assertEquals(List.of(accepted), stage.acceptedMeasurements());
        assertEquals(List.of(rejected), stage.rejectedMeasurements());
        PoseMeasurementSample output = (PoseMeasurementSample) stage.measurements().get(0);
        assertSame(accepted, output);
        assertEquals(accepted.stdDevs(), output.stdDevs());
        assertEquals(accepted.ambiguity(), output.ambiguity());
        assertEquals(accepted.targetCount(), output.targetCount());
        assertEquals(accepted.averageTargetDistanceMeters(), output.averageTargetDistanceMeters());
        assertSame(camera, output.source());
    }

    @Test
    void localizationStagesChainWithoutDiscardingTheIntermediatePose() {
        Sample old = sample(1.0, 0.0, 0.0, 1.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(0.5, 0.5, 0.5), new Object());
        Sample newest = sample(3.0, 2.0, 0.4, 3.0, 0.0, 2, 1.5,
                MeasurementStdDevs.of(0.2, 0.2, 0.2), new Object());
        Localization filtered = Localizations.filter()
                .input(signal(old, newest))
                .filter((localization, measurement, pose) -> pose.yMeters() >= 0.0)
                .name("filtered");
        Localization latest = Localizations.latestValid().input(filtered).name("latest");

        assertEquals(2, filtered.measurements().size());
        assertPose(latest.pose(), 3.0, 2.0, 0.4, EPSILON);
        assertSame(newest, latest.measurements().get(0));
        assertEquals("filtered", filtered.debugName());
        assertEquals("latest", latest.debugName());
    }

    @Test
    void latestValidSelectsNewestTimestampAndReportsAllAcceptedInputs() {
        Sample first = sample(1.0, 0.0, 0.0, 1.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(1.0, 1.0, 1.0), new Object());
        Sample latest = sample(3.0, 0.0, 0.0, 3.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(1.0, 1.0, 1.0), new Object());
        Sample middle = sample(2.0, 0.0, 0.0, 2.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(1.0, 1.0, 1.0), new Object());
        Localization localization = Localizations.latestValid().input(signal(first, latest, middle));

        assertSame(latest, localization.measurements().get(0));
        assertEquals(3, localization.acceptedMeasurements().size());
        assertTrue(localization.rejectedMeasurements().isEmpty());
    }

    @Test
    void weightedAverageUsesTranslationVarianceAndCircularHeadingAverage() {
        Sample weak = sample(0.0, 0.0, Math.toRadians(179.0), 4.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(2.0, 2.0, 0.4), new Object());
        Sample strong = sample(10.0, 4.0, Math.toRadians(-179.0), 6.0, 0.0, 1, 1.0,
                MeasurementStdDevs.of(1.0, 1.0, 0.2), new Object());
        Localization localization = Localizations.weightedAverage().input(signal(weak, strong));

        PoseMeasurementSample fused = (PoseMeasurementSample) localization.measurements().get(0);
        assertEquals(8.0, fused.pose().xMeters(), EPSILON);
        assertEquals(3.2, fused.pose().yMeters(), EPSILON);
        assertEquals(6.0, fused.timestampSeconds(), EPSILON);
        assertEquals(Math.sqrt(1.0 / 1.25), fused.stdDevs().xMeters(), EPSILON);
        assertEquals(0.2, fused.stdDevs().headingRadians(), EPSILON);
        assertTrue(Math.abs(Math.abs(fused.pose().headingRadians()) - Math.PI) < 0.02);
    }

    @Test
    void covarianceIntersectionGroupsSamplesAndReturnsConservativeCovariance() {
        Sample older = sample(1.0, 1.0, 0.1, 1.00, 0.0, 2, 2.0,
                MeasurementStdDevs.of(0.2, 1.0, 0.3), new Object());
        Sample newest = sample(3.0, 2.0, 0.2, 1.02, 0.0, 2, 2.0,
                MeasurementStdDevs.of(1.0, 0.2, 0.5), new Object());
        Sample stale = sample(20.0, 20.0, 1.0, 0.50, 0.0, 2, 2.0,
                MeasurementStdDevs.of(0.01, 0.01, 0.01), new Object());
        Localization localization = Localizations.covarianceIntersection()
                .input(signal(older, newest, stale))
                .groupWithinSeconds(0.03)
                .maxTranslationDisagreementMeters(3.0);

        PoseMeasurementSample fused = (PoseMeasurementSample) localization.measurements().get(0);
        assertTrue(fused.pose().xMeters() >= 1.0 && fused.pose().xMeters() <= 3.0);
        assertTrue(fused.pose().yMeters() >= 1.0 && fused.pose().yMeters() <= 2.0);
        assertEquals(1.02, fused.timestampSeconds(), EPSILON);
        assertTrue(Double.isFinite(fused.stdDevs().xMeters()));
        assertTrue(Double.isFinite(fused.stdDevs().yMeters()));
        assertTrue(fused.stdDevs().xMeters() >= 0.2);
        assertTrue(fused.stdDevs().yMeters() >= 0.2);
    }

    @Test
    void perSourceTagCovarianceAndDistanceScalingSurviveAFilterStage() {
        Object singleCamera = new Object();
        Object multiCamera = new Object();
        PoseSignal configured = signal(
                        sample(1.0, 2.0, 0.0, 1.0, 0.1, 1, 4.0,
                                MeasurementStdDevs.of(9.0, 9.0, 9.0), singleCamera),
                        sample(2.0, 3.0, 0.0, 2.0, 0.05, 3, 1.0,
                                MeasurementStdDevs.of(9.0, 9.0, 9.0), multiCamera))
                .singleTagStdDevs(0.5, 0.6, 0.7)
                .multiTagStdDevs(0.1, 0.2, 0.3)
                .distanceStdDevScaling(2.0, 2.0);
        Localization stage = Localizations.filter().input(configured);

        PoseMeasurementSample single = (PoseMeasurementSample) stage.measurements().get(0);
        PoseMeasurementSample multi = (PoseMeasurementSample) stage.measurements().get(1);
        assertEquals(2.0, single.stdDevs().xMeters(), EPSILON);
        assertEquals(2.4, single.stdDevs().yMeters(), EPSILON);
        assertEquals(2.8, single.stdDevs().headingRadians(), EPSILON);
        assertEquals(0.1, multi.stdDevs().xMeters(), EPSILON);
        assertEquals(0.2, multi.stdDevs().yMeters(), EPSILON);
        assertEquals(0.3, multi.stdDevs().headingRadians(), EPSILON);
        assertSame(singleCamera, single.source());
        assertSame(multiCamera, multi.source());
    }

    private static PoseSignal signal(Measurement... measurements) {
        List<Measurement> values = List.of(measurements);
        return () -> values;
    }

    private static Sample sample(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            double ambiguity,
            int targetCount,
            double averageDistanceMeters,
            MeasurementStdDevs stdDevs,
            Object source) {
        return new Sample(
                new PoseSnapshot(xMeters, yMeters, headingRadians),
                RobotVelocity.zero(),
                timestampSeconds,
                0.02,
                ambiguity,
                targetCount,
                averageDistanceMeters,
                stdDevs,
                source);
    }

    private static void assertPose(
            PoseSnapshot pose,
            double xMeters,
            double yMeters,
            double headingRadians,
            double tolerance) {
        assertEquals(xMeters, pose.xMeters(), tolerance);
        assertEquals(yMeters, pose.yMeters(), tolerance);
        assertEquals(headingRadians, pose.headingRadians(), tolerance);
    }

    private record Sample(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double latencySeconds,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters,
            MeasurementStdDevs stdDevs,
            Object source) implements PoseMeasurementSample {
    }
}
