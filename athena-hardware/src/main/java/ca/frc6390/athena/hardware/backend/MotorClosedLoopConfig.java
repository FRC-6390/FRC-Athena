package ca.frc6390.athena.hardware.backend;

/**
 * Vendor-neutral closed-loop gains for one motor-controller slot.
 */
public record MotorClosedLoopConfig(
        int slot,
        double p,
        double i,
        double d,
        double iZone,
        double tolerance,
        double staticFeedforward,
        double velocityFeedforward,
        double gravityFeedforward,
        FocPolicy focPolicy) {
    public MotorClosedLoopConfig {
        slot = Math.max(0, slot);
        p = finiteOrZero(p);
        i = finiteOrZero(i);
        d = finiteOrZero(d);
        iZone = finiteOrZero(iZone);
        tolerance = finiteOrZero(tolerance);
        staticFeedforward = finiteOrZero(staticFeedforward);
        velocityFeedforward = finiteOrZero(velocityFeedforward);
        gravityFeedforward = finiteOrZero(gravityFeedforward);
        focPolicy = focPolicy == null ? FocPolicy.DISABLED : focPolicy;
    }

    public static MotorClosedLoopConfig empty() {
        return new MotorClosedLoopConfig(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FocPolicy.DISABLED);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
