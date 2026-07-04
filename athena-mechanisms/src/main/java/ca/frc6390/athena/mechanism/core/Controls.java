package ca.frc6390.athena.mechanism.core;

/**
 * Factories for explicit command bindings.
 */
public final class Controls {
    private Controls() {
    }

    public static ControlRef percent() {
        return ControlRef.of(ControlKind.PERCENT);
    }

    public static ControlRef position() {
        return ControlRef.of(ControlKind.POSITION);
    }

    public static ControlRef velocity() {
        return ControlRef.of(ControlKind.VELOCITY);
    }

    public static ControlRef custom() {
        return ControlRef.of(ControlKind.CUSTOM);
    }
}
