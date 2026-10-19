package ca.frc6390.athena.runtime.measurement;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.List;
import org.junit.jupiter.api.Test;

class PoseSignalTest {
    @Test
    void appliesSingleAndMultiTagCovarianceThenDistanceScalingPerSample() {
        Object camera = new Object();
        PoseMeasurementSample single = Measurements.pose(new PoseSnapshot(1.0, 0.0, 0.0))
                .visionMetadata(0.15, 1, 4.0)
                .source(camera);
        PoseMeasurementSample multi = Measurements.pose(new PoseSnapshot(2.0, 0.0, 0.0))
                .visionMetadata(0.05, 3, 1.0)
                .source(camera);
        PoseSignal configured = ((PoseSignal) () -> List.of(single, multi))
                .singleTagStdDevs(Meters.of(0.7), Meters.of(0.8), Radians.of(0.9))
                .multiTagStdDevs(Meters.of(0.2), Meters.of(0.3), Radians.of(0.4))
                .distanceStdDevScaling(Meters.of(2.0), 2.0);

        PoseMeasurementSample configuredSingle = (PoseMeasurementSample) configured.measurements().get(0);
        PoseMeasurementSample configuredMulti = (PoseMeasurementSample) configured.measurements().get(1);

        assertEquals(2.8, configuredSingle.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(3.2, configuredSingle.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(3.6, configuredSingle.stdDevs().headingStdDev().in(Radians), 1.0e-9);
        assertEquals(0.2, configuredMulti.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(0.3, configuredMulti.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(0.4, configuredMulti.stdDevs().headingStdDev().in(Radians), 1.0e-9);
        assertSame(camera, configuredSingle.source());
        assertSame(camera, configuredMulti.source());
        assertEquals(0.15, configuredSingle.ambiguity(), 1.0e-9);
        assertEquals(3, configuredMulti.targetCount());
    }

    @Test
    void genericStdDevConfigurationWorksForAnyPoseSampleImplementation() {
        PoseMeasurementSample custom = new CustomPoseMeasurement();
        MeasurementSignal configured = ((MeasurementSignal) () -> List.of(custom)).stdDevs(Meters.of(0.4), Meters.of(0.5), Radians.of(0.6));

        PoseMeasurementSample result = (PoseMeasurementSample) configured.measurements().get(0);

        assertEquals(0.4, result.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(0.5, result.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(0.6, result.stdDevs().headingStdDev().in(Radians), 1.0e-9);
    }

    @Test
    void translationOnlyPreservesTranslationCovarianceAndDisablesHeadingFusion() {
        PoseMeasurementSample sample = Measurements.pose(new PoseSnapshot(1.0, 2.0, Math.PI));
        MeasurementSignal covariance = ((MeasurementSignal) () -> List.of(sample)).stdDevs(Meters.of(0.2), Meters.of(0.3), Radians.of(0.1));
        PoseSignal configured = PoseSignals.from(covariance).translationOnly();

        PoseMeasurementSample result = (PoseMeasurementSample) configured.measurements().get(0);

        assertEquals(0.2, result.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(0.3, result.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(1.0e6, result.stdDevs().headingStdDev().in(Radians), 1.0e-9);
        assertEquals(Math.PI, result.pose().headingRadians(), 1.0e-9);
    }

    @Test
    void translationOnlySurvivesLaterCovarianceConfiguration() {
        PoseMeasurementSample sample = Measurements.pose(new PoseSnapshot(1.0, 2.0, Math.PI))
                .visionMetadata(0.1, 2, 1.0);
        PoseSignal configured = ((PoseSignal) () -> List.of(sample))
                .translationOnly()
                .multiTagStdDevs(Meters.of(0.2), Meters.of(0.3), Radians.of(0.1));

        PoseMeasurementSample result = (PoseMeasurementSample) configured.measurements().get(0);

        assertEquals(0.2, result.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(0.3, result.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(1.0e6, result.stdDevs().headingStdDev().in(Radians), 1.0e-9);
    }

    @Test
    void headingOnlyPreservesHeadingCovarianceAndDisablesTranslationFusion() {
        PoseMeasurementSample sample = Measurements.pose(new PoseSnapshot(1.0, 2.0, 0.5))
                .visionMetadata(0.1, 2, 1.0);
        PoseSignal configured = ((PoseSignal) () -> List.of(sample))
                .headingOnly()
                .multiTagStdDevs(Meters.of(0.2), Meters.of(0.3), Radians.of(0.1));

        PoseMeasurementSample result = (PoseMeasurementSample) configured.measurements().get(0);

        assertEquals(1.0e6, result.stdDevs().xStdDev().in(Meters), 1.0e-9);
        assertEquals(1.0e6, result.stdDevs().yStdDev().in(Meters), 1.0e-9);
        assertEquals(0.1, result.stdDevs().headingStdDev().in(Radians), 1.0e-9);
    }

    private static final class CustomPoseMeasurement implements PoseMeasurementSample {
        @Override public PoseSnapshot pose() { return new PoseSnapshot(0.0, 0.0, 0.0); }
        @Override public ca.frc6390.athena.runtime.control.RobotVelocity speeds() {
            return ca.frc6390.athena.runtime.control.RobotVelocity.zero();
        }
        @Override public double timestampSeconds() { return 1.0; }
        @Override public double latencySeconds() { return 0.0; }
        @Override public double ambiguity() { return 0.0; }
        @Override public int targetCount() { return 1; }
        @Override public MeasurementStdDevs stdDevs() {
            return MeasurementStdDevs.of(Meters.of(1.0), Meters.of(1.0), Radians.of(1.0));
        }
        @Override public Object source() { return this; }
    }
}
