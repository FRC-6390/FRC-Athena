package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.input.InputType;
import java.util.Locale;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Reusable digital input declaration.
 */
public record DigitalInputRef(String name, int channel, boolean isInverted, BooleanSupplier reader) {
    /**
     * Creates a roboRIO digital input declaration.
     *
     * @param channel DIO channel
     * @return digital input ref
     */
    public static DigitalInputRef rio(int channel) {
        return named("dio" + channel, channel);
    }

    /**
     * Creates a named roboRIO digital input declaration.
     *
     * @param name input name
     * @param channel DIO channel
     * @return digital input ref
     */
    public static DigitalInputRef named(String name, int channel) {
        return new DigitalInputRef(name, channel, false, null);
    }

    public DigitalInputRef {
        name = name == null || name.isBlank() ? "dio" + channel : name;
    }

    /**
     * Marks this input as inverted.
     *
     * @return updated ref
     */
    public DigitalInputRef inverted() {
        return inverted(true);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true when active-low
     * @return updated ref
     */
    public DigitalInputRef inverted(boolean inverted) {
        return new DigitalInputRef(name, channel, inverted, reader);
    }

    /**
     * Binds a runtime reader to this declaration.
     *
     * @param reader reader
     * @return bound ref
     */
    public DigitalInputRef bind(BooleanSupplier reader) {
        return new DigitalInputRef(name, channel, isInverted, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads the raw digital value.
     *
     * @return raw value
     */
    public boolean raw() {
        if (reader == null) {
            throw new IllegalStateException("Digital input " + defaultName() + " is not runtime-bound.");
        }
        return reader.getAsBoolean();
    }

    /**
     * Reads the active value after inversion.
     *
     * @return active value
     */
    public boolean active() {
        boolean value = raw();
        return isInverted ? !value : value;
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
        return new InputSpec(ownerPath, defaultName(), InputType.BOOLEAN, InputSourceKind.DIGITAL_CHANNEL, channel, "");
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }
}
