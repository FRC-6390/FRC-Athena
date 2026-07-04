package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.input.InputType;
import java.util.Locale;
import java.util.Objects;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * Reusable numeric software signal declaration.
 */
public record NumberRef(String name, OptionalDouble defaultValue, DoubleSupplier reader) {
    /**
     * Creates a named numeric signal.
     *
     * @param name signal name
     * @return number ref
     */
    public static NumberRef of(String name) {
        return new NumberRef(name, OptionalDouble.empty(), null);
    }

    /**
     * Creates a named numeric signal with a default value.
     *
     * @param name signal name
     * @param defaultValue default value
     * @return number ref
     */
    public static NumberRef of(String name, double defaultValue) {
        return new NumberRef(name, OptionalDouble.of(defaultValue), null);
    }

    public NumberRef {
        name = name == null || name.isBlank() ? "number" : name;
        defaultValue = defaultValue == null ? OptionalDouble.empty() : defaultValue;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public NumberRef bind(DoubleSupplier reader) {
        return new NumberRef(name, defaultValue, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads this signal.
     *
     * @return value
     */
    public double value() {
        if (reader != null) {
            return reader.getAsDouble();
        }
        if (defaultValue.isPresent()) {
            return defaultValue.getAsDouble();
        }
        throw new IllegalStateException("Number signal " + defaultName() + " is not runtime-bound.");
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
        return new InputSpec(ownerPath, defaultName(), InputType.NUMBER, InputSourceKind.RUNTIME_SUPPLIER, -1, name);
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_').replace('/', '_');
    }
}
