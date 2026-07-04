package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Controller axis child ref.
 */
public record AxisRef(ControllerRef controller, String name, DoubleSupplier reader) {
    /**
     * Creates an axis ref.
     *
     * @param controller controller
     * @param name axis name
     * @return axis ref
     */
    public static AxisRef of(ControllerRef controller, String name) {
        return new AxisRef(controller, name, null);
    }

    public AxisRef {
        Objects.requireNonNull(controller, "controller");
        name = name == null || name.isBlank() ? "axis" : name;
    }

    /**
     * Binds a runtime reader.
     *
     * @param reader reader
     * @return bound ref
     */
    public AxisRef bind(DoubleSupplier reader) {
        return new AxisRef(controller, name, Objects.requireNonNull(reader, "reader"));
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
