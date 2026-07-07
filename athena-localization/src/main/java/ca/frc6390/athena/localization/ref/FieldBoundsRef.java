package ca.frc6390.athena.localization.ref;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;

/**
 * Rectangular field bounds that can be used as a localization filter.
 *
 * @param minX minimum x
 * @param minY minimum y
 * @param maxX maximum x
 * @param maxY maximum y
 */
public record FieldBoundsRef(double minX, double minY, double maxX, double maxY) implements LocalizationFilterRef {
    /**
     * Returns whether the pose is inside the bounds.
     *
     * @param pose pose
     * @return true when inside
     */
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
    public boolean accept(LocalizationFilterContext context) {
        return context.result()
                .map(result -> contains(result.pose()))
                .orElseGet(() -> context.measurement()
                        .filter(PoseMeasurement.class::isInstance)
                        .map(PoseMeasurement.class::cast)
                        .map(measurement -> contains(measurement.pose()))
                        .orElse(true));
    }
}
