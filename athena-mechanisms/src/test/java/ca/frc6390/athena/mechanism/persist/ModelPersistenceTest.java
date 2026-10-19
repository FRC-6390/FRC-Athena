package ca.frc6390.athena.mechanism.persist;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import ca.frc6390.athena.mechanism.model.ModelHistory;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import edu.wpi.first.units.Units;

/** Identified models, running estimates and history survive a restart through the persistent store. */
class ModelPersistenceTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final EncoderDevice ENCODER = EncoderDevice.of(EncoderKinds.CANCODER, 2)
            .measures(Units.Degrees);
    private static final String KEY = "turret/angle/characterization";

    @TempDir
    Path directory;

    @Test
    void modelRunningEstimateAndHistoryRoundTripThroughTheStore() throws IOException {
        MechanismModel model = new MechanismModel(MotionKind.ROTARY, 0.5, 0.4, 2.0, 0.2, 0.3, 0.7,
                new FrictionTable(List.of(0.0, 0.5), List.of(0.1, 0.2)), Double.NaN, 0.04, 0.05,
                0.0125, 0.02, 2, List.of("kraken_x60", "kraken_x44"), 10.0, Map.of(), 0.98, 240);
        MechanismModel running = model.withPayloadDelta(edu.wpi.first.units.Units.Volts.of(0.0));

        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        storage.save(KEY, model);
        storage.saveRunning(KEY, running);
        storage.saveHistory(KEY, List.of(new ModelHistory.Snapshot(12.0, "disable", running)));
        assertTrue(store.flush());

        PersistentStore reopened = PersistentStore.open(directory);
        PersistentModelStorage restored = new PersistentModelStorage(reopened);
        assertEquals(model, restored.load(KEY).orElseThrow());
        assertEquals(running, restored.loadRunning(KEY).orElseThrow());
        List<ModelHistory.Snapshot> history = restored.loadHistory(KEY);
        assertEquals(1, history.size());
        assertEquals("disable", history.get(0).reason());
        assertEquals(12.0, history.get(0).timestampSeconds(), 0.0);
        assertEquals(running.kV(), history.get(0).model().kV(), 0.0);
    }

    @Test
    void keysCoverEverythingAStorageWrites() {
        Set<String> keys = PersistentModelStorage.keys(KEY);
        assertTrue(keys.contains(KEY + "/model"));
        assertTrue(keys.contains(KEY + "/running"));
        assertTrue(keys.contains(KEY + "/history"));
        assertEquals(9, keys.size());
    }

    @Test
    void emptyHistoryClearsTheStoredHistory() throws IOException {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.LINEAR);
        storage.saveHistory(KEY, List.of(new ModelHistory.Snapshot(1.0, "match", model)));
        storage.saveHistory(KEY, List.of());
        assertTrue(storage.loadHistory(KEY).isEmpty());
        assertFalse(store.get(KEY + "/history").isPresent());
    }

    @Test
    void characterizationRestoresActiveModelAndWritesThroughOnFit() throws IOException {
        MechanismModel stored = MechanismModel.empty(MotionKind.ROTARY).withMotors(0.02, 0.1, 1, List.of("k"), 5.0);
        PersistentStore first = PersistentStore.open(directory);
        new PersistentModelStorage(first).save(KEY, new MechanismModel(MotionKind.ROTARY, 0.3, 0.3, 1.5, 0.1, 0.0,
                0.0, FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0, Map.of(), 0.0, 0));
        first.flush();

        PersistentStore reopened = PersistentStore.open(directory);
        Characterization characterization = Characterization.of(Controls.position(MOTOR).feedback(ENCODER))
                .persistAs(new PersistentModelStorage(reopened), KEY);
        assertEquals(1.5, characterization.model().kV(), 0.0, "boot restores the persisted model");
        assertEquals(characterization.model(), characterization.baseline());
        assertEquals(KEY, characterization.storageKey());

        characterization.recordMatchEnd();
        assertTrue(new PersistentModelStorage(reopened).loadHistory(KEY).stream()
                .anyMatch(snapshot -> snapshot.reason().equals("match")));
        assertFalse(stored.equals(characterization.model()));
    }
}
