package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import java.util.Objects;

/**
 * A built-in filter that computes its rejection reason in the same check that decides acceptance, so the reason
 * is never out of step with the decision.
 */
final class ReasonedFilter implements LocalizationFilter {
    /** Returns null to accept, or the rejection reason. */
    @FunctionalInterface
    interface Check {
        String rejection(Localization localization, Measurement measurement, PoseSnapshot pose);
    }

    private final String name;
    private final Check check;

    ReasonedFilter(String name, Check check) {
        this.name = Objects.requireNonNull(name, "name");
        this.check = Objects.requireNonNull(check, "check");
    }

    /** Null when accepted, otherwise the reason. */
    String rejection(Localization localization, Measurement measurement, PoseSnapshot pose) {
        return check.rejection(localization, measurement, pose);
    }

    @Override
    public boolean accept(Localization localization, Measurement measurement, PoseSnapshot pose) {
        return check.rejection(localization, measurement, pose) == null;
    }

    @Override
    public String rejectionReason(Localization localization, Measurement measurement, PoseSnapshot pose) {
        String reason = check.rejection(localization, measurement, pose);
        return reason == null ? "" : reason;
    }

    @Override
    public String name() {
        return name;
    }
}
