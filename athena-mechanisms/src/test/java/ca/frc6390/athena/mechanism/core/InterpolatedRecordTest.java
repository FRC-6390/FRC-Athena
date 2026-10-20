package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class InterpolatedRecordTest {
    @TempDir
    Path directory;

    private final List<String> markers = new ArrayList<>();
    private int fullCaptureRequests;

    @AfterEach
    void restoreMarkers() {
        RuntimeMarkers.install(null);
    }

    @Test
    void recordingInsertsThePointAndPlantsAMarkerWithMetadata() {
        installRecorder();
        CurveMechanism mechanism = new CurveMechanism();
        mechanism.input[0] = 2.0;

        mechanism.curve.recordCurrent();

        assertEquals(3, mechanism.curve.size());
        assertEquals(20.0, mechanism.curve.target(), 1.0e-9);
        assertEquals(1, fullCaptureRequests);
        assertEquals(1, markers.size());
        assertTrue(markers.get(0).contains("input=2.0"), markers.get(0));
        assertTrue(markers.get(0).contains("output=20.0"), markers.get(0));
        assertTrue(markers.get(0).contains("matchType="), markers.get(0));
        assertTrue(markers.get(0).contains("replayId=none"), markers.get(0));
        assertEquals(1.0, mechanism.curve.telemetry().get("Recorded/Count").number());
    }

    @Test
    void recordingTheSameInputAgainReplacesThePoint() {
        CurveMechanism mechanism = new CurveMechanism();
        mechanism.input[0] = 2.0;
        mechanism.curve.recordCurrent();
        mechanism.input[0] = 2.0;
        mechanism.curve.recordCurrent();
        assertEquals(3, mechanism.curve.size());
    }

    @Test
    void recordActionRunsThroughTheSchedulerAndPersistsAcrossRegistrations() {
        installRecorder();
        PersistentStore store = PersistentStore.open(directory);
        CurveMechanism first = new CurveMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .persistentStore(store, null)
                .register(first)
                .bindInMemoryRuntime();
        scheduler.telemetrySchema();
        first.input[0] = 2.5;
        scheduler.request(first.record);
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(3, first.curve.size());
        assertEquals(1, markers.size());
        scheduler.robotPeriodic(0.02, 0.02);
        assertEquals(1, PersistentStore.open(directory).keys().size());

        CurveMechanism second = new CurveMechanism();
        MechanismScheduler reloaded = MechanismScheduler.create()
                .persistentStore(PersistentStore.open(directory), null)
                .register(second)
                .bindInMemoryRuntime();
        reloaded.telemetrySchema();
        assertEquals(3, second.curve.size(), "recorded points are restored");
        assertEquals(1.0, second.curve.telemetry().get("Recorded/Count").number());
    }

    @Test
    void unpersistedRecordingIsNotStored() {
        PersistentStore store = PersistentStore.open(directory);
        UnpersistedMechanism mechanism = new UnpersistedMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .persistentStore(store, null)
                .register(mechanism)
                .bindInMemoryRuntime();
        scheduler.telemetrySchema();
        mechanism.input[0] = 2.0;
        mechanism.curve.recordCurrent();
        scheduler.robotPeriodic(0.0, 0.02);
        assertEquals(0, store.keys().size());
        assertEquals(3, mechanism.curve.size());
    }

    private void installRecorder() {
        RuntimeMarkers.install(new RuntimeMarkers() {
            @Override
            public void mark(int kind, String message) {
                if (kind == RuntimeMarkers.RECORD) {
                    markers.add(message);
                }
            }

            @Override
            public void requestFullCapture() {
                fullCaptureRequests++;
            }
        });
    }

    static final class CurveMechanism implements Mechanism {
        final double[] input = {1.0};
        final ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 7), Units.Rotations);
        final InterpolatedControlAction curve = control
                .interpolate(InterpolationKinds.LINEAR, () -> input[0], Units.Value)
                .at(Units.Value.of(1.0), Units.Rotations.of(10.0))
                .at(Units.Value.of(3.0), Units.Rotations.of(30.0))
                .persisted();
        final Action record = curve.record();
    }

    static final class UnpersistedMechanism implements Mechanism {
        final double[] input = {1.0};
        final ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 8), Units.Rotations);
        final InterpolatedControlAction curve = control
                .interpolate(InterpolationKinds.LINEAR, () -> input[0], Units.Value)
                .at(Units.Value.of(1.0), Units.Rotations.of(10.0))
                .at(Units.Value.of(3.0), Units.Rotations.of(30.0));
    }
}
