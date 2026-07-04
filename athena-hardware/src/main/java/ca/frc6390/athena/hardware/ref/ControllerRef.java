package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;

/**
 * Reusable controller declaration.
 */
public record ControllerRef(String kind, int port) {
    /**
     * Creates an Xbox-style controller declaration.
     *
     * @param port driver station port
     * @return controller ref
     */
    public static ControllerRef xbox(int port) {
        return new ControllerRef("xbox", port);
    }

    public ControllerRef {
        kind = kind == null || kind.isBlank() ? "controller" : kind;
    }

    /**
     * Creates an axis child ref.
     *
     * @param name axis name
     * @return axis ref
     */
    public AxisRef axis(String name) {
        return AxisRef.of(this, name);
    }

    /**
     * Creates a button child ref.
     *
     * @param name button name
     * @return button ref
     */
    public ButtonRef button(String name) {
        return ButtonRef.of(this, name);
    }

    public AxisRef leftX() {
        return axis("leftX");
    }

    public AxisRef leftY() {
        return axis("leftY");
    }

    public AxisRef rightX() {
        return axis("rightX");
    }

    public AxisRef rightY() {
        return axis("rightY");
    }

    public AxisRef leftTrigger() {
        return axis("leftTrigger");
    }

    public AxisRef rightTrigger() {
        return axis("rightTrigger");
    }

    public ButtonRef leftBumper() {
        return button("leftBumper");
    }

    public ButtonRef rightBumper() {
        return button("rightBumper");
    }

    public ButtonRef a() {
        return button("a");
    }

    public ButtonRef b() {
        return button("b");
    }

    public ButtonRef x() {
        return button("x");
    }

    public ButtonRef y() {
        return button("y");
    }

    /**
     * Returns a threshold button from an axis.
     *
     * @param axis axis
     * @param threshold active threshold
     * @return button ref
     */
    public ButtonRef threshold(AxisRef axis, double threshold) {
        Objects.requireNonNull(axis, "axis");
        return ButtonRef.of(this, axis.name() + "At" + threshold).bind(() -> axis.value() >= threshold);
    }

    /**
     * Returns stable controller name.
     *
     * @return default name
     */
    public String defaultName() {
        return sanitize(kind) + "_" + port;
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(' ', '_').replace('-', '_');
    }
}
