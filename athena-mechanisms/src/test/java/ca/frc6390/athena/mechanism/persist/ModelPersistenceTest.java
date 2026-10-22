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
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
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
        Characterization characterization = Characterization.of(Controls.position(MOTOR, Units.Degrees).feedback(ENCODER))
                .persistAs(new PersistentModelStorage(reopened), KEY);
        assertEquals(1.5, characterization.model().kV(), 0.0, "boot restores the persisted model");
        assertEquals(characterization.model(), characterization.baseline());
        assertEquals(KEY, characterization.storageKey());

        characterization.recordMatchEnd();
        assertTrue(new PersistentModelStorage(reopened).loadHistory(KEY).stream()
                .anyMatch(snapshot -> snapshot.reason().equals("match")));
        assertFalse(stored.equals(characterization.model()));
    }

    @Test
    void unknownStoredMotionKindsKeepCodeDefaultsDuringMechanismRegistration() {
        PersistentStore store = PersistentStore.open(directory);
        String key = "persistedTurret/characterization";
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.ROTARY);
        storage.save(key, model);
        storage.saveRunning(key, model);
        storage.saveHistory(key, List.of(new ModelHistory.Snapshot(1.0, "match", model)));
        store.put(key + "/model.motion", "OLD_ROTARY_KIND");
        store.put(key + "/running.motion", "OLD_ROTARY_KIND");
        store.put(key + "/history.motion", "OLD_ROTARY_KIND");
        assertTrue(store.flush());

        PersistentStore reopened = PersistentStore.open(directory);
        assertEquals(0, reopened.skippedLines(), "the store syntax is valid; the model metadata is not");
        PersistedTurret turret = new PersistedTurret();
        MechanismModel defaults = turret.characterization.model();
        MechanismScheduler.create().persistentStore(reopened, null).register(turret);

        assertEquals(defaults, turret.characterization.model());
        assertEquals(defaults, turret.characterization.baseline());
        assertEquals(defaults, turret.characterization.runningEstimate());
        assertTrue(turret.characterization.history().snapshots().isEmpty());
        assertEquals(key, turret.characterization.storageKey());
        assertEquals("OLD_ROTARY_KIND", reopened.get(key + "/model.motion").orElseThrow(),
                "a rejected read does not destroy the original data");
    }

    @Test
    void malformedActiveModelDoesNotPreventValidRunningEstimateAndHistoryLoads() {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.ROTARY);
        storage.save(KEY, model);
        storage.saveRunning(KEY, model);
        ModelHistory.Snapshot snapshot = new ModelHistory.Snapshot(1.0, "match", model);
        storage.saveHistory(KEY, List.of(snapshot));
        double[] vector = ModelCodec.encode(model);
        vector[12] = 0.0;
        store.put(KEY + "/model", vector);

        assertTrue(storage.load(KEY).isEmpty());
        assertEquals(model, storage.loadRunning(KEY).orElseThrow());
        assertEquals(List.of(snapshot), storage.loadHistory(KEY));
    }

    @Test
    void storedMotionKindMustMatchTheCurrentControlDeclaration() {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel incompatible = MechanismModel.empty(MotionKind.LINEAR);
        storage.save(KEY, incompatible);
        storage.saveRunning(KEY, incompatible);
        storage.saveHistory(KEY, List.of(new ModelHistory.Snapshot(1.0, "old linear declaration", incompatible)));

        Characterization characterization = Characterization.of(Controls.position(MOTOR, Units.Degrees).feedback(ENCODER))
                .defaults(MechanismModel.empty(MotionKind.ROTARY).withMotors(0.02, 0.1, 1, List.of("k"), 5.0));
        MechanismModel defaults = characterization.model();
        characterization.persistAs(storage, KEY);

        assertEquals(defaults, characterization.model());
        assertEquals(defaults, characterization.baseline());
        assertEquals(defaults, characterization.runningEstimate());
        assertTrue(characterization.history().snapshots().isEmpty());
    }

    @Test
    void malformedCountsAndGearRatiosAreRejectedWithoutTruncationOrOverflow() {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.LINEAR);
        storage.save(KEY, model);
        int[] fields = {11, 14, 16};
        double[] invalidCounts = {-1.0, 0.5, (double) Integer.MAX_VALUE + 1.0, Double.MAX_VALUE};
        for (int field : fields) {
            for (double count : invalidCounts) {
                double[] vector = ModelCodec.encode(model);
                vector[field] = count;
                store.put(KEY + "/model", vector);
                assertTrue(storage.load(KEY).isEmpty(), "field " + field + " count " + count);
            }
        }
        for (double gear : new double[] {0.0, -1.0}) {
            double[] vector = ModelCodec.encode(model);
            vector[12] = gear;
            store.put(KEY + "/model", vector);
            assertTrue(storage.load(KEY).isEmpty(), "gear " + gear);
        }
        for (double mask : new double[] {-1.0, 0.5, 4.0}) {
            double[] vector = ModelCodec.encode(model);
            vector[15] = mask;
            store.put(KEY + "/model", vector);
            assertTrue(storage.load(KEY).isEmpty(), "mask " + mask);
        }
    }

    @Test
    void missingOrIncorrectMetadataTypesAreRejected() {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.ROTARY);
        storage.save(KEY, model);
        store.remove(KEY + "/model.motion");
        assertTrue(storage.load(KEY).isEmpty());
        store.put(KEY + "/model.motion", true);
        assertTrue(storage.load(KEY).isEmpty());
        store.put(KEY + "/model.motion", "ROTARY");
        store.put(KEY + "/model.kinds", 5L);
        assertTrue(storage.load(KEY).isEmpty());
        store.put(KEY + "/model.kinds", "");
        store.put(KEY + "/model", "invalid vector type");
        assertTrue(storage.load(KEY).isEmpty());

        store.put(KEY + "/history", new double[] {1.0, 17.0});
        store.put(KEY + "/history.reasons", 5L);
        store.put(KEY + "/history.motion", "ROTARY");
        assertTrue(storage.loadHistory(KEY).isEmpty());
    }

    @Test
    void malformedHistoryLengthsAndModelsKeepValidSnapshots() {
        PersistentStore store = PersistentStore.open(directory);
        PersistentModelStorage storage = new PersistentModelStorage(store);
        MechanismModel model = MechanismModel.empty(MotionKind.ROTARY);
        ModelHistory.Snapshot snapshot = new ModelHistory.Snapshot(1.0, "valid", model);
        storage.saveHistory(KEY, List.of(snapshot));
        double[] valid = (double[]) store.get(KEY + "/history").orElseThrow();

        for (double length : new double[] {-1.0, 0.5, Integer.MAX_VALUE, Double.MAX_VALUE}) {
            double[] combined = java.util.Arrays.copyOf(valid, valid.length + 2);
            combined[valid.length] = 2.0;
            combined[valid.length + 1] = length;
            store.put(KEY + "/history", combined);
            store.put(KEY + "/history.reasons", "valid\ninvalid");
            assertEquals(List.of(snapshot), storage.loadHistory(KEY), "length " + length);
        }
        double[] combined = new double[valid.length * 3];
        System.arraycopy(valid, 0, combined, 0, valid.length);
        System.arraycopy(valid, 0, combined, valid.length, valid.length);
        System.arraycopy(valid, 0, combined, valid.length * 2, valid.length);
        combined[valid.length + 2 + 12] = -1.0; // Invalid middle model: gear ratio.
        store.put(KEY + "/history", combined);
        store.put(KEY + "/history.reasons", "valid\ninvalid\nvalid");
        assertEquals(List.of(snapshot, snapshot), storage.loadHistory(KEY));

        store.put(KEY + "/history", new double[] {1.0});
        store.put(KEY + "/history.reasons", "truncated");
        assertTrue(storage.loadHistory(KEY).isEmpty());
    }

    static final class PersistedTurret implements Mechanism {
        public final Characterization characterization = Characterization.of(
                Controls.position(MOTOR, Units.Degrees).feedback(ENCODER))
                .defaults(MechanismModel.empty(MotionKind.ROTARY).withMotors(0.02, 0.1, 1, List.of("k"), 5.0));
    }
}
