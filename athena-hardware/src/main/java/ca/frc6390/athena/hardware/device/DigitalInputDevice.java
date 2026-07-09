package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.BooleanSupplier;

/**
 * Reusable digital input declaration.
 */
public record DigitalInputDevice(String name, int channel, boolean isInverted, BooleanSupplier reader) {
    private static final ConcurrentMap<DigitalInputDevice, BooleanSupplier> RUNTIME_READERS = new ConcurrentHashMap<>();

    /**
     * Creates a roboRIO digital input declaration.
     *
     * @param channel DIO channel
     * @return digital input ref
     */
    public static DigitalInputDevice rio(int channel) {
        return new DigitalInputDevice("dio" + channel, channel, false, null);
    }

    public DigitalInputDevice {
        name = name == null || name.isBlank() ? "dio" + channel : name;
    }

    /**
     * Marks this input as inverted.
     *
     * @return updated ref
     */
    public DigitalInputDevice inverted() {
        return inverted(true);
    }

    /**
     * Sets inversion.
     *
     * @param inverted true when active-low
     * @return updated ref
     */
    public DigitalInputDevice inverted(boolean inverted) {
        return new DigitalInputDevice(name, channel, inverted, reader);
    }

    /**
     * Binds a runtime reader to this declaration.
     *
     * @param reader reader
     * @return bound ref
     */
    public DigitalInputDevice bind(BooleanSupplier reader) {
        return new DigitalInputDevice(name, channel, isInverted, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Binds a runtime reader to an existing ref instance.
     *
     * @param ref input ref
     * @param reader runtime reader
     */
    public static void bindRuntime(DigitalInputDevice ref, BooleanSupplier reader) {
        RUNTIME_READERS.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads the raw digital value.
     *
     * @return raw value
     */
    public boolean raw() {
        if (reader == null) {
            BooleanSupplier runtimeReader = RUNTIME_READERS.get(this);
            if (runtimeReader == null) {
                throw new IllegalStateException("Digital input " + defaultName() + " is not runtime-bound.");
            }
            return runtimeReader.getAsBoolean();
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

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }
}
