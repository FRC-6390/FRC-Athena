package ca.frc6390.athena.localization.ref;

/**
 * Factories for localization field bounds.
 */
public final class FieldBounds {
    private FieldBounds() {
    }

    /**
     * Creates rectangular bounds.
     *
     * @param minX minimum x
     * @param minY minimum y
     * @param maxX maximum x
     * @param maxY maximum y
     * @return bounds
     */
    public static FieldBoundsRef of(double minX, double minY, double maxX, double maxY) {
        return new FieldBoundsRef(minX, minY, maxX, maxY);
    }

    /**
     * Creates full-field bounds from dimensions.
     *
     * @param lengthMeters field length
     * @param widthMeters field width
     * @return bounds
     */
    public static FieldBoundsRef field(double lengthMeters, double widthMeters) {
        return of(0.0, 0.0, lengthMeters, widthMeters);
    }
}
