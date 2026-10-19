package ca.frc6390.athena.runtime.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

import static edu.wpi.first.units.Units.Meters;

/** A 2D shape with containment, distance, and expansion operations. */
@FunctionalInterface
public interface Geometry2d {
    /**
     * Returns the signed distance to this geometry's boundary in meters. Values are negative inside,
     * zero on the boundary, and positive outside. Allocation-free; intended for runtime hot paths.
     */
    double signedDistancePrimitive(Translation2d point);

    /** Returns the signed distance to this geometry's boundary. Negative inside, positive outside. */
    default Distance signedDistance(Translation2d point) {
        return Meters.of(signedDistancePrimitive(Objects.requireNonNull(point, "point")));
    }

    /** Returns whether a point is inside or on this geometry. */
    default boolean contains(Translation2d point) {
        return signedDistancePrimitive(Objects.requireNonNull(point, "point")) <= 0.0;
    }

    /** Returns whether a pose's translation is inside or on this geometry. */
    default boolean contains(Pose2d pose) {
        return pose != null && contains(pose.getTranslation());
    }

    /** Returns this geometry expanded outward by the supplied distance. */
    default Geometry2d expanded(Distance distance) {
        double meters = Objects.requireNonNull(distance, "distance").in(Meters);
        if (!Double.isFinite(meters)) {
            throw new IllegalArgumentException("Geometry expansion must be finite.");
        }
        return point -> signedDistancePrimitive(point) - meters;
    }
}
