package ca.frc6390.athena.hardware.ref;

/**
 * Reusable gear ratio declaration.
 *
 * @param ratio multiplier from sensor units to mechanism units
 */
public record GearRatioRef(double ratio) {
    public GearRatioRef {
        if (!Double.isFinite(ratio) || ratio == 0.0) {
            throw new IllegalArgumentException("Gear ratio must be finite and non-zero.");
        }
    }

    /**
     * Creates a sensor-to-mechanism ratio.
     *
     * @param ratio ratio
     * @return gear ratio ref
     */
    public static GearRatioRef sensorToMechanism(double ratio) {
        return new GearRatioRef(ratio);
    }

    /**
     * Creates a reduction from motor rotations to mechanism rotations.
     *
     * @param motorRotations motor rotations
     * @param mechanismRotations mechanism rotations
     * @return gear ratio ref
     */
    public static GearRatioRef reduction(double motorRotations, double mechanismRotations) {
        if (mechanismRotations == 0.0) {
            throw new IllegalArgumentException("Mechanism rotations must be non-zero.");
        }
        return new GearRatioRef(mechanismRotations / motorRotations);
    }
}
