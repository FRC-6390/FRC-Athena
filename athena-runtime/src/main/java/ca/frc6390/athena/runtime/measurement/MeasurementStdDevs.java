package ca.frc6390.athena.runtime.measurement;

/**
 * Standard deviations for a field-relative pose measurement.
 *
 * @param xMeters x standard deviation
 * @param yMeters y standard deviation
 * @param headingRadians heading standard deviation
 */
public record MeasurementStdDevs(double xMeters, double yMeters, double headingRadians) {
    /**
     * Creates pose standard deviations.
     *
     * @param xMeters x standard deviation
     * @param yMeters y standard deviation
     * @param headingRadians heading standard deviation
     * @return standard deviations
     */
    public static MeasurementStdDevs of(double xMeters, double yMeters, double headingRadians) {
        return new MeasurementStdDevs(xMeters, yMeters, headingRadians);
    }

    public MeasurementStdDevs {
        xMeters = positiveOrInfinite(xMeters);
        yMeters = positiveOrInfinite(yMeters);
        headingRadians = positiveOrInfinite(headingRadians);
    }

    /**
     * Returns average translation variance.
     *
     * @return translation variance
     */
    public double translationVariance() {
        return (xMeters * xMeters + yMeters * yMeters) / 2.0;
    }

    private static double positiveOrInfinite(double value) {
        if (!Double.isFinite(value)) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(1.0e-9, value);
    }
}
