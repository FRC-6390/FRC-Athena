package ca.frc6390.athena.hardware.device;

/** Effective current limits configured for a motor. */
public record MotorCurrentLimits(int supplyAmps, int statorAmps) {
    public MotorCurrentLimits {
        if (supplyAmps < 0 || statorAmps < 0) {
            throw new IllegalArgumentException("Motor current limits cannot be negative.");
        }
    }

    /** Returns the best current limit for detecting a stalled motor. */
    public int stallDetectionAmps() {
        return statorAmps > 0 ? statorAmps : supplyAmps;
    }

    /** Returns whether stall detection can derive a threshold from these limits. */
    public boolean supportsStallDetection() {
        return stallDetectionAmps() > 0;
    }
}
