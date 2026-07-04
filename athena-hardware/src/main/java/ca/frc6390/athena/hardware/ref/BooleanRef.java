package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.input.InputType;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Reusable boolean software signal declaration.
 */
public record BooleanRef(String name, Optional<Boolean> defaultValue, BooleanSupplier reader) {
    /**
     * Creates a named boolean signal.
     *
     * @param name signal name
     * @return boolean ref
     */
    public static BooleanRef of(String name) {
        return new BooleanRef(name, Optional.empty(), null);
    }

    /**
     * Creates a named boolean signal with a default value.
     *
     * @param name signal name
     * @param defaultValue default value
     * @return boolean ref
     */
    public static BooleanRef of(String name, boolean defaultValue) {
        return new BooleanRef(name, Optional.of(defaultValue), null);
    }

    public BooleanRef {
        name = name == null || name.isBlank() ? "boolean" : name;
        defaultValue = defaultValue == null ? Optional.empty() : defaultValue;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public BooleanRef bind(BooleanSupplier reader) {
        return new BooleanRef(name, defaultValue, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads this signal.
     *
     * @return value
     */
    public boolean active() {
        if (reader != null) {
            return reader.getAsBoolean();
        }
        if (defaultValue.isPresent()) {
            return defaultValue.get();
        }
        throw new IllegalStateException("Boolean signal " + defaultName() + " is not runtime-bound.");
    }

    /**
     * Returns a stable default name.
     *
     * @return default name
     */
    public String defaultName() {
        return sanitize(name);
    }

    /**
     * Lowers this ref to an input spec.
     *
     * @param ownerPath owning path
     * @return input spec
     */
    public InputSpec toSpec(String ownerPath) {
        return new InputSpec(ownerPath, defaultName(), InputType.BOOLEAN, InputSourceKind.RUNTIME_SUPPLIER, -1, name);
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_').replace('/', '_');
    }
}
