package ca.frc6390.athena.core.auto;

/**
 * Simple PID constants used for holonomic path following. Keeps the core vendor-agnostic while
 * still providing enough information for adapters to build vendor-specific controllers.
 */
public record HolonomicPidConstants(double kP, double kI, double kD, double iZone) {
    public HolonomicPidConstants {
        // Avoid NaN propagation while letting zero values pass through.
        kP = Double.isFinite(kP) ? kP : 0.0;
        kI = Double.isFinite(kI) ? kI : 0.0;
        kD = Double.isFinite(kD) ? kD : 0.0;
        iZone = Double.isFinite(iZone) ? iZone : 0.0;
    }

    public HolonomicPidConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0);
    }
}
