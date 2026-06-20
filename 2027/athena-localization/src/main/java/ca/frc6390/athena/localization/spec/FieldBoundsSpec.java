package ca.frc6390.athena.localization.spec;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Rectangular field bounds for accepted poses.
 *
 * @param name bounds name
 * @param minXMeters minimum x in meters
 * @param minYMeters minimum y in meters
 * @param maxXMeters maximum x in meters
 * @param maxYMeters maximum y in meters
 */
public record FieldBoundsSpec(
        String name,
        double minXMeters,
        double minYMeters,
        double maxXMeters,
        double maxYMeters) {
    /**
     * Creates an unbounded field region.
     *
     * @return unbounded region
     */
    public static FieldBoundsSpec unbounded() {
        return new FieldBoundsSpec("unbounded", Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public FieldBoundsSpec {
        name = name == null || name.isBlank() ? "field" : name;
    }

    /**
     * Returns true when the bounds describe a finite rectangle.
     *
     * @return true for finite bounds
     */
    public boolean isFinite() {
        return Double.isFinite(minXMeters)
                && Double.isFinite(minYMeters)
                && Double.isFinite(maxXMeters)
                && Double.isFinite(maxYMeters);
    }

    /**
     * Returns true when minimums are less than or equal to maximums.
     *
     * @return true for ordered bounds
     */
    public boolean isOrdered() {
        return minXMeters <= maxXMeters && minYMeters <= maxYMeters;
    }

    /**
     * Returns true when the pose is inside these bounds.
     *
     * @param pose pose snapshot
     * @return true if contained
     */
    public boolean contains(PoseSnapshot pose) {
        return pose != null
                && pose.xMeters() >= minXMeters
                && pose.xMeters() <= maxXMeters
                && pose.yMeters() >= minYMeters
                && pose.yMeters() <= maxYMeters;
    }
}
