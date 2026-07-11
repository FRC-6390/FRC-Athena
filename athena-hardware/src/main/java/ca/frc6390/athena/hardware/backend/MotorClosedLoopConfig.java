package ca.frc6390.athena.hardware.backend;

/**
 * Vendor-neutral voltage-output closed-loop gains for one motor-controller slot.
 * PID gains produce volts from Athena position or velocity error units, and
 * feedforward gains produce volts from the corresponding reference units.
 */
public record MotorClosedLoopConfig(
        int slot,
        double p,
        double i,
        double d,
        double iZone,
        double staticFeedforward,
        double velocityFeedforward,
        double accelerationFeedforward,
        double gravityFeedforward,
        FocPolicy focPolicy) {
    public MotorClosedLoopConfig {
        slot = Math.max(0, slot);
        requireFinite(p, "Proportional gain");
        requireFinite(i, "Integral gain");
        requireFinite(d, "Derivative gain");
        if (!Double.isFinite(iZone) || iZone < 0.0) {
            throw new IllegalArgumentException("Integral zone must be finite and non-negative.");
        }
        requireFinite(staticFeedforward, "Static feedforward");
        requireFinite(velocityFeedforward, "Velocity feedforward");
        requireFinite(accelerationFeedforward, "Acceleration feedforward");
        requireFinite(gravityFeedforward, "Gravity feedforward");
        focPolicy = focPolicy == null ? FocPolicy.DISABLED : focPolicy;
    }

    public static MotorClosedLoopConfig empty() {
        return new MotorClosedLoopConfig(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FocPolicy.DISABLED);
    }

    private static void requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
    }
}
