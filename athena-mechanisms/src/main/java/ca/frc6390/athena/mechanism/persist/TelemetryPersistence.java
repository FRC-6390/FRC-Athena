package ca.frc6390.athena.mechanism.persist;

import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.RuntimeMarkers;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * Connects persisted telemetry values to a {@link PersistentStore}.
 *
 * <p>The schema builder calls {@link #bind} for every persisted writable value and {@link #bindArray}
 * for persisted recorded data. Binding restores the stored value over the code default. The robot loop
 * calls {@link #periodic} once per cycle: it detects changes made by code or by dashboard writes,
 * writes them to the store, reports each change through {@link RuntimeMarkers}, and flushes the store
 * at most once per second. While disabled, pending changes are flushed at once.</p>
 */
public final class TelemetryPersistence {
    private static final TelemetryPersistence DISABLED = new TelemetryPersistence(null, () -> Double.NaN);

    private final PersistentStore store;
    private final DoubleSupplier matchTimeSeconds;
    private final List<Binding> bindings = new ArrayList<>();
    private final Map<String, Binding> bindingsByKey = new HashMap<>();
    private final Set<String> boundKeys = new HashSet<>();
    private boolean pruned;

    /**
     * Creates a persistence context.
     *
     * @param store store, or null to disable persistence
     * @param matchTimeSeconds supplier of the FMS match time reported with writes (NaN when unknown)
     */
    public TelemetryPersistence(PersistentStore store, DoubleSupplier matchTimeSeconds) {
        this.store = store;
        this.matchTimeSeconds = matchTimeSeconds == null ? () -> Double.NaN : matchTimeSeconds;
    }

    /** Returns a context that persists nothing. */
    public static TelemetryPersistence disabled() {
        return DISABLED;
    }

    /** Returns true when a store is attached. */
    public boolean enabled() {
        return store != null;
    }

    /**
     * Keeps store entries owned by declarations that are not bound telemetry values (for example persisted models),
     * so the boot-time cleanup does not delete them. Call before the first schema build.
     *
     * @param keys store keys to keep
     */
    public void keep(Set<String> keys) {
        if (store != null) {
            boundKeys.addAll(keys);
        }
    }

    /** Returns the attached store, or null when disabled. */
    public PersistentStore store() {
        return store;
    }

    /**
     * Restores a persisted writable value and tracks it for writes. Values that are not writable or not
     * marked persisted are returned unchanged. Numeric, boolean, and string values are supported.
     *
     * @param key telemetry path used as the store key
     * @param value value to bind
     * @return the same value
     */
    public TelemetryValue bind(String key, TelemetryValue value) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(value, "value");
        if (store == null || !value.isPersisted() || !value.writable()) {
            return value;
        }
        boundKeys.add(key);
        Object stored = store.get(key).orElse(null);
        try {
            switch (value.type()) {
                case NUMBER -> {
                    if (stored instanceof Double number) value.set(number);
                }
                case BOOLEAN -> {
                    if (stored instanceof Boolean flag) value.set(flag);
                }
                case STRING -> {
                    if (stored instanceof String text) value.set(text);
                }
                default -> throw new IllegalArgumentException("Persisted telemetry supports number, boolean, and string values.");
            }
        } catch (RuntimeException rejected) {
            // A stored value the declaration no longer accepts keeps the code default.
        }
        register(new ValueBinding(key, value));
        return value;
    }

    /**
     * Restores and tracks a persisted array. The reader is evaluated only when {@code version} changes.
     *
     * @param key store key
     * @param version monotonically increasing version of the array contents
     * @param reader current contents
     * @param writer applies restored contents
     */
    public void bindArray(String key, LongSupplier version, Supplier<double[]> reader, Consumer<double[]> writer) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(version, "version");
        Objects.requireNonNull(reader, "reader");
        Objects.requireNonNull(writer, "writer");
        if (store == null) {
            return;
        }
        boundKeys.add(key);
        store.get(key).ifPresent(stored -> {
            if (stored instanceof double[] array) {
                try {
                    writer.accept(array);
                } catch (RuntimeException rejected) {
                    // Invalid stored data keeps the code default.
                }
            }
        });
        register(new ArrayBinding(key, version, reader));
    }

    /**
     * Removes stored entries that no bound declaration owns. Runs once, after the first schema build,
     * so values for renamed or removed fields do not linger.
     *
     * @return number of removed entries
     */
    public int pruneUnbound() {
        if (store == null || pruned) {
            return 0;
        }
        pruned = true;
        int removed = store.retainOnly(Set.copyOf(boundKeys));
        if (removed > 0) {
            store.flush();
        }
        return removed;
    }

    /**
     * Detects changes to bound values, records them, and flushes the store.
     *
     * @param nowSeconds robot timestamp
     * @param mode current lifecycle mode, reported with each write
     * @param enabled true while enabled; pending changes are flushed immediately when disabled
     */
    public void periodic(double nowSeconds, LifecycleMode mode, boolean enabled) {
        if (store == null) {
            return;
        }
        for (int index = 0, size = bindings.size(); index < size; index++) {
            bindings.get(index).sync(nowSeconds, mode);
        }
        if (!enabled) {
            if (store.dirty()) {
                store.flush();
            }
        } else {
            store.flushIfDue(nowSeconds);
        }
    }

    /** Tracks a binding; rebinding the same key (after a schema rebuild) replaces the old binding. */
    private void register(Binding binding) {
        Binding previous = bindingsByKey.put(binding.key, binding);
        if (previous == null) {
            bindings.add(binding);
        } else {
            bindings.set(bindings.indexOf(previous), binding);
        }
    }

    private void report(String key, Object oldValue, Object newValue, double nowSeconds, LifecycleMode mode) {
        RuntimeMarkers.current().mark(RuntimeMarkers.PERSISTED_WRITE,
                "path=" + key + " old=" + oldValue + " new=" + newValue
                        + " time=" + nowSeconds + " mode=" + mode + " matchTime=" + matchTimeSeconds.getAsDouble());
    }

    private abstract static class Binding {
        final String key;

        Binding(String key) {
            this.key = key;
        }

        abstract void sync(double nowSeconds, LifecycleMode mode);
    }

    private final class ValueBinding extends Binding {
        private final TelemetryValue value;
        private double lastNumber;
        private boolean lastBoolean;
        private String lastText;

        ValueBinding(String key, TelemetryValue value) {
            super(key);
            this.value = value;
            remember();
        }

        private void remember() {
            switch (value.type()) {
                case NUMBER -> lastNumber = value.number();
                case BOOLEAN -> lastBoolean = value.bool();
                default -> lastText = String.valueOf(value.value());
            }
        }

        @Override
        void sync(double nowSeconds, LifecycleMode mode) {
            switch (value.type()) {
                case NUMBER -> {
                    double current = value.number();
                    if (Double.compare(current, lastNumber) != 0 && Double.isFinite(current)) {
                        store.put(key, current);
                        report(key, lastNumber, current, nowSeconds, mode);
                        lastNumber = current;
                    }
                }
                case BOOLEAN -> {
                    boolean current = value.bool();
                    if (current != lastBoolean) {
                        store.put(key, current);
                        report(key, lastBoolean, current, nowSeconds, mode);
                        lastBoolean = current;
                    }
                }
                default -> {
                    String current = String.valueOf(value.value());
                    if (!current.equals(lastText)) {
                        store.put(key, current);
                        report(key, lastText, current, nowSeconds, mode);
                        lastText = current;
                    }
                }
            }
        }
    }

    private final class ArrayBinding extends Binding {
        private final LongSupplier version;
        private final Supplier<double[]> reader;
        private long lastVersion;

        ArrayBinding(String key, LongSupplier version, Supplier<double[]> reader) {
            super(key);
            this.version = version;
            this.reader = reader;
            this.lastVersion = version.getAsLong();
        }

        @Override
        void sync(double nowSeconds, LifecycleMode mode) {
            long current = version.getAsLong();
            if (current == lastVersion) {
                return;
            }
            lastVersion = current;
            double[] contents = reader.get();
            store.put(key, contents);
            report(key, "version " + (current - 1), "version " + current, nowSeconds, mode);
        }
    }
}
