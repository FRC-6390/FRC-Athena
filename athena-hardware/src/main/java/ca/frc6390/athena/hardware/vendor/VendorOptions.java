package ca.frc6390.athena.hardware.vendor;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Typed vendor-specific declaration metadata.
 *
 * @param options option instances keyed by option class
 */
public record VendorOptions(Map<Class<?>, Object> options) {
    /**
     * Returns an empty option bag.
     *
     * @return empty options
     */
    public static VendorOptions empty() {
        return new VendorOptions(Map.of());
    }

    public VendorOptions {
        options = options == null ? Map.of() : Map.copyOf(options);
    }

    /**
     * Returns a new option bag with one option value.
     *
     * @param type option class
     * @param value option value
     * @param <T> option type
     * @return updated options
     */
    public <T> VendorOptions with(Class<T> type, T value) {
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(value, "value");
        Map<Class<?>, Object> updated = new LinkedHashMap<>(options);
        updated.put(type, type.cast(value));
        return new VendorOptions(updated);
    }

    /**
     * Finds an option by type.
     *
     * @param type option class
     * @param <T> option type
     * @return option if present
     */
    public <T> Optional<T> find(Class<T> type) {
        Objects.requireNonNull(type, "type");
        return Optional.ofNullable(options.get(type)).map(type::cast);
    }
}
