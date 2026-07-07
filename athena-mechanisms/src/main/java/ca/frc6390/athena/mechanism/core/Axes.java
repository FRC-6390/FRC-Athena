package ca.frc6390.athena.mechanism.core;

/**
 * Factories for explicit axis bindings.
 */
public final class Axes {
    private Axes() {
    }

    public static AxisRef percent() {
        return AxisRef.of(AxisKind.PERCENT);
    }

    public static AxisRef voltage() {
        return AxisRef.of(AxisKind.VOLTAGE);
    }

    public static AxisRef position() {
        return AxisRef.of(AxisKind.POSITION);
    }

    public static AxisRef velocity() {
        return AxisRef.of(AxisKind.VELOCITY);
    }

    public static AxisRef custom() {
        return AxisRef.of(AxisKind.CUSTOM);
    }
}
