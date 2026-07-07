package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Controller axis child ref.
 */
public record ControllerAxisRef(ControllerRef controller, String name, DoubleSupplier reader) {
    /**
     * Creates a controller axis ref.
     *
     * @param controller controller
     * @param name axis name
     * @return controller axis ref
     */
    public static ControllerAxisRef of(ControllerRef controller, String name) {
        return new ControllerAxisRef(controller, name, null);
    }

    public ControllerAxisRef {
        Objects.requireNonNull(controller, "controller");
        name = name == null || name.isBlank() ? "axis" : name;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public ControllerAxisRef bind(DoubleSupplier reader) {
        return new ControllerAxisRef(controller, name, Objects.requireNonNull(reader, "reader"));
    }

    /**
     * Reads this axis.
     *
     * @return axis value
     */
    public double value() {
        if (reader == null) {
            throw new IllegalStateException("Controller axis " + defaultName() + " is not runtime-bound.");
        }
        return reader.getAsDouble();
    }

    /**
     * Returns a button ref that is active when this axis exceeds a threshold.
     *
     * @param threshold active threshold
     * @return button ref
     */
    public ButtonRef at(double threshold) {
        return controller.threshold(this, threshold);
    }

    /**
     * Returns stable axis name.
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
