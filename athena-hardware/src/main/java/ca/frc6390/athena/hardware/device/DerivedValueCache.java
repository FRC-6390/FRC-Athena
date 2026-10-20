package ca.frc6390.athena.hardware.device;

import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;

/**
 * Bounded, thread-safe cache of values derived from immutable declarations.
 *
 * <p>Declarations are records, so they cannot hold extra fields. Values derived from their components, such as
 * default names, are cached here so per-cycle callers do not rebuild them. Keys are compared with
 * {@code equals}, which is safe because the derived value depends only on the key's components. Once the cache
 * holds {@code maxEntries} values, further misses are computed without being stored, so code that creates many
 * short-lived declarations cannot grow it without bound.</p>
 *
 * @param <K> declaration type
 * @param <V> derived value type
 */
final class DerivedValueCache<K, V> {
    private final ConcurrentHashMap<K, V> values = new ConcurrentHashMap<>();
    private final Function<? super K, ? extends V> compute;
    private final int maxEntries;

    DerivedValueCache(Function<? super K, ? extends V> compute, int maxEntries) {
        this.compute = Objects.requireNonNull(compute, "compute");
        this.maxEntries = maxEntries;
    }

    V get(K key) {
        V value = values.get(key);
        if (value != null) {
            return value;
        }
        value = Objects.requireNonNull(compute.apply(key), "derived value");
        if (values.size() >= maxEntries) {
            return value;
        }
        V previous = values.putIfAbsent(key, value);
        return previous != null ? previous : value;
    }
}
