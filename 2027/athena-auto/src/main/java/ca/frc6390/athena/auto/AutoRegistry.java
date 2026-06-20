package ca.frc6390.athena.auto;

import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Registry of installed autonomous routine providers.
 */
public final class AutoRegistry {
    private static final AutoRegistry GLOBAL = new AutoRegistry();

    private final Map<String, AutoSource> sources = new ConcurrentHashMap<>();

    /**
     * Returns the shared registry.
     *
     * @return global registry
     */
    public static AutoRegistry get() {
        return GLOBAL;
    }

    /**
     * Registers an auto source.
     *
     * @param key source key
     * @param source auto source
     * @return this registry
     */
    public AutoRegistry register(String key, AutoSource source) {
        sources.put(normalizeKey(key), Objects.requireNonNull(source, "source"));
        return this;
    }

    /**
     * Looks up a source without throwing.
     *
     * @param key source key
     * @return source if registered
     */
    public Optional<AutoSource> find(String key) {
        return Optional.ofNullable(sources.get(normalizeKey(key)));
    }

    /**
     * Looks up a source or throws with dependency guidance.
     *
     * @param key source key
     * @return registered source
     */
    public AutoSource require(String key) {
        String normalized = normalizeKey(key);
        AutoSource source = sources.get(normalized);
        if (source == null) {
            throw new MissingAutoSourceException(normalized);
        }
        return source;
    }

    /**
     * Removes all registered sources.
     */
    public void clear() {
        sources.clear();
    }

    private static String normalizeKey(String key) {
        return key == null || key.isBlank() ? "default" : key.trim();
    }
}
