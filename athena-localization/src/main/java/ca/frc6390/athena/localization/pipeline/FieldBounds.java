package ca.frc6390.athena.localization.pipeline;

/**
 * Factories for localization field bounds.
 */
public final class FieldBounds {
    private FieldBounds() {
    }

    public static FieldBoundsFilter of(double minX, double minY, double maxX, double maxY) {
        return new FieldBoundsFilter(minX, minY, maxX, maxY);
    }

    public static FieldBoundsFilter field(double lengthMeters, double widthMeters) {
        return of(0.0, 0.0, lengthMeters, widthMeters);
    }
}
