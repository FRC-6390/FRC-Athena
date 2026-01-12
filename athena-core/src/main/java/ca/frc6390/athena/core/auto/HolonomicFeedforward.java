package ca.frc6390.athena.core.auto;

/**
 * Vendor-neutral feedforward values for holonomic drives.
 */
public record HolonomicFeedforward(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    public static HolonomicFeedforward zero() {
        return new HolonomicFeedforward(0.0, 0.0, 0.0);
    }
}
