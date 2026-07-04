package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.input.InputType;
import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Reusable analog input declaration.
 */
public record AnalogInputRef(String name, int channel, DoubleSupplier reader) {
    /**
     * Creates a roboRIO analog input declaration.
     *
     * @param channel analog channel
     * @return analog input ref
     */
    public static AnalogInputRef rio(int channel) {
        return named("analog" + channel, channel);
    }

    /**
     * Creates a named roboRIO analog input declaration.
     *
     * @param name input name
     * @param channel analog channel
     * @return analog input ref
     */
    public static AnalogInputRef named(String name, int channel) {
        return new AnalogInputRef(name, channel, null);
    }

    public AnalogInputRef {
        name = name == null || name.isBlank() ? "analog" + channel : name;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public AnalogInputRef bind(DoubleSupplier reader) {
        return new AnalogInputRef(name, channel, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads the analog voltage/value.
     *
     * @return value
     */
    public double value() {
        if (reader == null) {
            throw new IllegalStateException("Analog input " + defaultName() + " is not runtime-bound.");
        }
        return reader.getAsDouble();
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
        return new InputSpec(ownerPath, defaultName(), InputType.NUMBER, InputSourceKind.ANALOG_CHANNEL, channel, "");
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }
}
