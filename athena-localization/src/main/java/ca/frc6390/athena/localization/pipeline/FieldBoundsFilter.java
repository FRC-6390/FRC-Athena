package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

/**
 * Rectangular field bounds that can be used as a localization filter.
 */
public record FieldBoundsFilter(double minX, double minY, double maxX, double maxY) implements LocalizationFilter {
    public boolean contains(PoseSnapshot pose) {
        if (pose == null) {
            return false;
        }
        return pose.xMeters() >= Math.min(minX, maxX)
                && pose.xMeters() <= Math.max(minX, maxX)
                && pose.yMeters() >= Math.min(minY, maxY)
                && pose.yMeters() <= Math.max(minY, maxY);
    }

    @Override
    public boolean accept(Localization localization, Measurement measurement, PoseSnapshot pose) {
        if (pose != null) {
            return contains(pose);
        }
        return !(measurement instanceof PoseMeasurementSample sample) || contains(sample.pose());
    }
}
