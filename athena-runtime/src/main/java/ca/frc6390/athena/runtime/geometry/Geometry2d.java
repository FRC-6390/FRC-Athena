package ca.frc6390.athena.runtime.geometry;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Objects;

/** A 2D shape with containment, distance, and expansion operations. */
@FunctionalInterface
public interface Geometry2d {
    /**
     * Returns signed distance to this geometry's boundary. Values are negative
     * inside, zero on the boundary, and positive outside.
     */
    double signedDistance(Point2d point);

    /** Returns whether a point is inside or on this geometry. */
    default boolean contains(Point2d point) {
        return signedDistance(Objects.requireNonNull(point, "point")) <= 0.0;
    }

    /** Returns whether a pose's translation is inside or on this geometry. */
    default boolean contains(PoseSnapshot pose) {
        return pose != null && contains(Point2d.from(pose));
    }

    /** Returns this geometry expanded by the supplied distance. */
    default Geometry2d expanded(double distance) {
        if (!Double.isFinite(distance)) {
            throw new IllegalArgumentException("Geometry expansion must be finite.");
        }
        return point -> signedDistance(point) - distance;
    }

}
