package ca.frc6390.athena.runtime.measurement;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.runtime.control.ModifiedAxis;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

class MeasurementSignalPolicyTest {
    @Test
    void chainedPoliciesFilterLatencyAgeAndDisabledState() {
        Measurement fresh = timedPose(9.5, 0.04);
        Measurement stale = timedPose(8.0, 0.04);
        Measurement slow = timedPose(9.8, 0.20);
        MeasurementSignal signal = Measurements.measurements(() -> List.of(fresh, stale, slow))
                .latencyLimit(0.05)
                .maxAge(10.0, 1.0)
                .enabled(() -> true);

        assertEquals(List.of(fresh), signal.measurements());
        assertEquals(fresh, signal.latestMeasurement().orElseThrow());

        assertTrue(signal.enabled(() -> false).measurements().isEmpty());
    }

    @Test
    void stdDevsApplyToPoseMeasurements() {
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.3, 0.4);
        MeasurementSignal signal = Measurements.measurement(() -> timedPose(1.0, 0.0)).stdDevs(stdDevs);

        Measurement measurement = signal.latestMeasurement().orElseThrow();
        PoseMeasurementSample pose = assertInstanceOf(PoseMeasurementSample.class, measurement);
        assertEquals(stdDevs, pose.stdDevs());
    }

    @Test
    void stdDevValuesCanBeConfiguredWithoutASettingsObject() {
        MeasurementSignal signal = Measurements.measurement(() -> timedPose(1.0, 0.0)).stdDevs(0.2, 0.4);

        PoseMeasurementSample pose = assertInstanceOf(
                PoseMeasurementSample.class,
                signal.latestMeasurement().orElseThrow());
        assertEquals(MeasurementStdDevs.of(0.2, 0.2, 0.4), pose.stdDevs());
    }

    @Test
    void measurementSnapshotCachesListAndLatestUntilRefresh() {
        AtomicReference<List<Measurement>> values = new AtomicReference<>(List.of(timedPose(1.0, 0.0)));
        MeasurementSnapshot snapshot = new MeasurementSnapshot(values::get);

        snapshot.refresh();
        values.set(List.of(timedPose(2.0, 0.0)));

        assertEquals(1.0, snapshot.latestMeasurement().orElseThrow().timestampSeconds(), 1.0e-9);
        assertEquals(1.0, snapshot.measurements().get(0).timestampSeconds(), 1.0e-9);

        snapshot.refresh();

        assertEquals(2.0, snapshot.latestMeasurement().orElseThrow().timestampSeconds(), 1.0e-9);
        assertEquals(2.0, snapshot.measurements().get(0).timestampSeconds(), 1.0e-9);
    }

    @Test
    void measurementSnapshotSharesAnImmutableCycleView() {
        Measurement first = timedPose(1.0, 0.0);
        Measurement latest = timedPose(2.0, 0.0);
        List<Measurement> values = List.of(first, latest);
        MeasurementSnapshot.CycleView cycle = new MeasurementSnapshot.CycleView() {
            @Override
            public List<Measurement> measurements() {
                return values;
            }

            @Override
            public java.util.Optional<Measurement> latestMeasurement() {
                return java.util.Optional.of(latest);
            }
        };
        MeasurementSnapshot snapshot = new MeasurementSnapshot(cycle).refresh();

        assertSame(values, snapshot.measurements());
        assertSame(latest, snapshot.latestMeasurement().orElseThrow());
    }

    @Test
    void modifiedAxisDeadzonesClampsSquaresAndInverts() {
        double[] raw = {0.6};
        ModifiedAxis axis = new ModifiedAxis(() -> raw[0], 0.2).squared(true).inverted(true);

        assertEquals(-0.25, axis.getAsDouble(), 1.0e-9);

        raw[0] = 0.1;
        assertEquals(0.0, axis.getAsDouble(), 1.0e-9);

        raw[0] = Double.NaN;
        assertEquals(0.0, axis.getAsDouble(), 1.0e-9);
    }

    @Test
    void robotVelocityConvertsFieldRelativeAndClampsMagnitude() {
        RobotVelocity robot = RobotVelocity.field(2.0, 0.0, 3.0).fieldToRobot(Math.PI / 2.0);

        assertEquals(0.0, robot.xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, robot.yMetersPerSecond(), 1.0e-9);
        assertEquals(3.0, robot.angularRadiansPerSecond(), 1.0e-9);

        RobotVelocity clamped = new RobotVelocity(3.0, 4.0, -5.0).clamp(2.5, 1.25);
        assertEquals(1.5, clamped.xMetersPerSecond(), 1.0e-9);
        assertEquals(2.0, clamped.yMetersPerSecond(), 1.0e-9);
        assertEquals(-1.25, clamped.angularRadiansPerSecond(), 1.0e-9);
    }

    private static Measurement timedPose(double timestampSeconds, double latencySeconds) {
        return ((PoseMeasurement) Measurements.pose(new PoseSnapshot(timestampSeconds, 0.0, 0.0)))
                .timing(timestampSeconds, latencySeconds);
    }
}
