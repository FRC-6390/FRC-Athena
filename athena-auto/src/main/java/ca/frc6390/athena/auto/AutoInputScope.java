package ca.frc6390.athena.auto;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Typed autonomous inputs for one routine scope.
 */
public final class AutoInputScope {
    private final Map<String, Supplier<String>> strings = new LinkedHashMap<>();
    private final Map<String, BooleanSupplier> booleans = new LinkedHashMap<>();
    private final Map<String, DoubleSupplier> doubles = new LinkedHashMap<>();

    /**
     * Sets a string input value.
     *
     * @param key input key
     * @param value input value
     * @return this scope
     */
    public AutoInputScope string(String key, String value) {
        return stringSupplier(key, () -> value);
    }

    /**
     * Sets a string input supplier.
     *
     * @param key input key
     * @param value input value supplier
     * @return this scope
     */
    public AutoInputScope stringSupplier(String key, Supplier<String> value) {
        strings.put(normalizeKey(key), Objects.requireNonNull(value, "value"));
        return this;
    }

    /**
     * Sets a boolean input value.
     *
     * @param key input key
     * @param value input value
     * @return this scope
     */
    public AutoInputScope bool(String key, boolean value) {
        return boolSupplier(key, () -> value);
    }

    /**
     * Sets a boolean input supplier.
     *
     * @param key input key
     * @param value input value supplier
     * @return this scope
     */
    public AutoInputScope boolSupplier(String key, BooleanSupplier value) {
        booleans.put(normalizeKey(key), Objects.requireNonNull(value, "value"));
        return this;
    }

    /**
     * Sets a double input value.
     *
     * @param key input key
     * @param value input value
     * @return this scope
     */
    public AutoInputScope number(String key, double value) {
        return numberSupplier(key, () -> value);
    }

    /**
     * Sets a double input supplier.
     *
     * @param key input key
     * @param value input value supplier
     * @return this scope
     */
    public AutoInputScope numberSupplier(String key, DoubleSupplier value) {
        doubles.put(normalizeKey(key), Objects.requireNonNull(value, "value"));
        return this;
    }

    /**
     * Reads a string input.
     *
     * @param key input key
     * @return string value if present
     */
    public Optional<String> findString(String key) {
        Supplier<String> value = strings.get(normalizeKey(key));
        return value == null ? Optional.empty() : Optional.ofNullable(value.get());
    }

    /**
     * Reads a string input or fallback.
     *
     * @param key input key
     * @param fallback fallback value
     * @return input or fallback
     */
    public String readString(String key, String fallback) {
        return findString(key).orElse(fallback);
    }

    /**
     * Reads a boolean input.
     *
     * @param key input key
     * @return boolean value if present
     */
    public Optional<Boolean> findBool(String key) {
        BooleanSupplier value = booleans.get(normalizeKey(key));
        return value == null ? Optional.empty() : Optional.of(value.getAsBoolean());
    }

    /**
     * Reads a boolean input or fallback.
     *
     * @param key input key
     * @param fallback fallback value
     * @return input or fallback
     */
    public boolean readBool(String key, boolean fallback) {
        return findBool(key).orElse(fallback);
    }

    /**
     * Reads a double input.
     *
     * @param key input key
     * @return double value if present
     */
    public Optional<Double> findNumber(String key) {
        DoubleSupplier value = doubles.get(normalizeKey(key));
        return value == null ? Optional.empty() : Optional.of(value.getAsDouble());
    }

    /**
     * Reads a double input or fallback.
     *
     * @param key input key
     * @param fallback fallback value
     * @return input or fallback
     */
    public double readNumber(String key, double fallback) {
        return findNumber(key).orElse(fallback);
    }

    /**
     * Removes one input key across all types.
     *
     * @param key input key
     * @return this scope
     */
    public AutoInputScope clear(String key) {
        String normalized = normalizeKey(key);
        strings.remove(normalized);
        booleans.remove(normalized);
        doubles.remove(normalized);
        return this;
    }

    /**
     * Removes every input in this scope.
     *
     * @return this scope
     */
    public AutoInputScope clear() {
        strings.clear();
        booleans.clear();
        doubles.clear();
        return this;
    }

    private static String normalizeKey(String key) {
        return key == null || key.isBlank() ? "input" : key.trim();
    }
}
