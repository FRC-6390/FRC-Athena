package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Controller button child ref.
 */
public record ButtonRef(ControllerRef controller, String name, BooleanSupplier reader) {
    /**
     * Creates a button ref.
     *
     * @param controller controller
     * @param name button name
     * @return button ref
     */
    public static ButtonRef of(ControllerRef controller, String name) {
        return new ButtonRef(controller, name, null);
    }

    public ButtonRef {
        Objects.requireNonNull(controller, "controller");
        name = name == null || name.isBlank() ? "button" : name;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public ButtonRef bind(BooleanSupplier reader) {
        return new ButtonRef(controller, name, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads this button.
     *
     * @return true when pressed
     */
    public boolean pressed() {
        if (reader == null) {
            throw new IllegalStateException("Controller button " + defaultName() + " is not runtime-bound.");
        }
        return reader.getAsBoolean();
    }

    /**
     * Alias for pressed.
     *
     * @return true when active
     */
    public boolean active() {
        return pressed();
    }

    /**
     * Returns stable button name.
     *
     * @return default name
     */
    public String defaultName() {
        return controller.defaultName() + "_" + sanitize(name);
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }
}
