package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.persist.Locks;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import ca.frc6390.athena.mechanism.persist.TuningLocks;
import java.nio.file.Path;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PersistentTelemetryTest {
    @TempDir
    Path directory;

    @Test
    void persistedValueIsSavedOnChangeAndRestoredOnNextRegistration() {
        PersistedMechanism first = new PersistedMechanism();
        PersistentStore store = PersistentStore.open(directory);
        MechanismScheduler scheduler = MechanismScheduler.create()
                .persistentStore(store, null)
                .register(first)
                .bindInMemoryRuntime();
        TelemetryValue speed = valueEndingWith(scheduler.telemetrySchema(), "/Values/speed");
        assertTrue(speed.isPersisted());
        assertEquals(5.0, speed.number());

        speed.set(42.0);
        scheduler.robotPeriodic(0.0, 0.02);
        String key = keyEndingWith(scheduler.telemetrySchema(), "/Values/speed");
        assertEquals(42.0, store.getDouble(key).orElseThrow());
        assertEquals(42.0, PersistentStore.open(directory).getDouble(key).orElseThrow());

        PersistedMechanism second = new PersistedMechanism();
        MechanismScheduler reloaded = MechanismScheduler.create()
                .persistentStore(PersistentStore.open(directory), null)
                .register(second)
                .bindInMemoryRuntime();
        reloaded.telemetrySchema();
        assertEquals(42.0, second.speed, "the stored value overrides the code default");
        assertEquals("saved", second.label);
    }

    @Test
    void storedKeysWithoutAnOwnerAreDeletedAtBoot() {
        PersistentStore store = PersistentStore.open(directory);
        store.put("removed/field", 1.0);
        store.flush();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .persistentStore(PersistentStore.open(directory), null)
                .register(new PersistedMechanism())
                .bindInMemoryRuntime();
        scheduler.telemetrySchema();
        assertFalse(PersistentStore.open(directory).get("removed/field").isPresent());
    }

    @Test
    void unpublishedValuesStayLocalButPersist() {
        MechanismScheduler scheduler = MechanismScheduler.create()
                .persistentStore(PersistentStore.open(directory), null)
                .register(new PersistedMechanism())
                .bindInMemoryRuntime();
        TelemetryValue hidden = valueEndingWith(scheduler.telemetrySchema(), "/Values/hidden");
        assertFalse(hidden.isPublished());
        assertTrue(hidden.isPersisted());
        assertTrue(valueEndingWith(scheduler.telemetrySchema(), "/Values/speed").isPublished());
    }

    @Test
    void lockGroupsRejectWritesServerSide() {
        PersistedMechanism mechanism = new PersistedMechanism();
        TuningLocks locks = new TuningLocks();
        AtomicBoolean matchLocked = new AtomicBoolean(false);
        locks.define(Locks.InMatch.class, matchLocked::get);
        MechanismScheduler scheduler = MechanismScheduler.create()
                .tuningLocks(locks)
                .register(mechanism)
                .bindInMemoryRuntime();
        TelemetryValue speed = valueEndingWith(scheduler.telemetrySchema(), "/Values/speed");
        TelemetryValue open = valueEndingWith(scheduler.telemetrySchema(), "/Values/open");
        assertEquals(Locks.InMatch.class, speed.lockGroup());
        assertFalse(speed.locked());

        matchLocked.set(true);
        assertTrue(speed.locked());
        TelemetryValue.WriteResult rejected = speed.tryWrite(10.0);
        assertFalse(rejected.accepted());
        assertEquals("rejected: locked (InMatch)", rejected.reason());
        assertEquals(5.0, mechanism.speed, "a rejected write is not applied");

        assertTrue(open.tryWrite(0.5).accepted(), "Locks.Never is never locked");
        assertEquals(0.5, mechanism.open);

        matchLocked.set(false);
        assertTrue(speed.tryWrite(10.0).accepted());
        assertEquals(10.0, mechanism.speed);
    }

    @Test
    void readOnlyValuesRejectWrites() {
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(new PersistedMechanism())
                .bindInMemoryRuntime();
        TelemetryValue constant = TelemetryValue.constant(1.0);
        assertEquals("rejected: read-only", constant.tryWrite(2.0).reason());
    }

    private static TelemetryValue valueEndingWith(TelemetrySchema schema, String suffix) {
        return schema.values().entrySet().stream()
                .filter(entry -> entry.getKey().endsWith(suffix))
                .map(Map.Entry::getValue)
                .findFirst()
                .orElseThrow();
    }

    private static String keyEndingWith(TelemetrySchema schema, String suffix) {
        return schema.values().keySet().stream()
                .filter(key -> key.endsWith(suffix))
                .findFirst()
                .orElseThrow();
    }

    static final class PersistedMechanism implements Mechanism {
        @Telemetry(persist = true, writable = true, min = 0.0, max = 100.0)
        double speed = 5.0;

        @Telemetry(persist = true, writable = true)
        String label = "saved";

        @Telemetry(persist = true, writable = true, publish = false)
        boolean hidden;

        @Telemetry(writable = true, lock = Locks.Never.class)
        double open = 1.0;
    }
}
