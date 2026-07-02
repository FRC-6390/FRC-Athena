package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.mechanism.spec.PidSpec;

/**
 * Reusable PID declaration for robot constants.
 */
public record PidRef(double p, double i, double d, double iZone, double tolerance) {
    public static PidRef of(double p, double i, double d) {
        return new PidRef(p, i, d, 0.0, 0.0);
    }

    public PidRef iZone(double iZone) {
        return new PidRef(p, i, d, iZone, tolerance);
    }

    public PidRef tolerance(double tolerance) {
        return new PidRef(p, i, d, iZone, tolerance);
    }

    public PidSpec toSpec() {
        return new PidSpec(p, i, d, iZone, tolerance);
    }
}
