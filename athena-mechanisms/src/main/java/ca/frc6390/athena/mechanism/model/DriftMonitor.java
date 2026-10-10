package ca.frc6390.athena.mechanism.model;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Compares a running estimate with a baseline. A parameter drifts when
 * {@code |running - baseline| / max(|baseline|, floor)} exceeds the tolerance band.
 */
public final class DriftMonitor {
    /** Relative tolerance used when none is configured. */
    public static final double DEFAULT_TOLERANCE = 0.25;
    private static final double FLOOR = 1.0e-3;

    /**
     * Result of one comparison.
     *
     * @param alert whether any compared parameter left the band
     * @param message human-readable summary, empty when there is no alert
     */
    public record DriftReport(boolean alert, String message) {
        public static final DriftReport NONE = new DriftReport(false, "");
    }

    private DriftMonitor() {
    }

    /** Compares the compared parameters of two models. */
    public static DriftReport evaluate(MechanismModel baseline, MechanismModel running, double tolerance) {
        if (baseline == null || running == null) {
            return DriftReport.NONE;
        }
        if (!(tolerance > 0.0) || !Double.isFinite(tolerance)) {
            throw new IllegalArgumentException("Drift tolerance must be positive and finite.");
        }
        List<String> drifted = new ArrayList<>();
        compare(drifted, "kineticFrictionVolts", baseline.kineticFrictionVolts(), running.kineticFrictionVolts(), tolerance);
        compare(drifted, "kV", baseline.kV(), running.kV(), tolerance);
        compare(drifted, "kA", baseline.kA(), running.kA(), tolerance);
        compare(drifted, "kG", baseline.kG(), running.kG(), tolerance);
        if (drifted.isEmpty()) {
            return DriftReport.NONE;
        }
        return new DriftReport(true, "Model drift outside " + format(tolerance * 100.0) + "%: "
                + String.join("; ", drifted));
    }

    private static void compare(List<String> drifted, String name, double baseline, double running,
            double tolerance) {
        if (!Double.isFinite(baseline) || !Double.isFinite(running)) {
            return;
        }
        double relative = Math.abs(running - baseline) / Math.max(Math.abs(baseline), FLOOR);
        if (relative > tolerance) {
            drifted.add(name + " " + format(baseline) + " -> " + format(running)
                    + " (" + format(relative * 100.0) + "%)");
        }
    }

    private static String format(double value) {
        return String.format(Locale.ROOT, "%.4g", value);
    }
}
