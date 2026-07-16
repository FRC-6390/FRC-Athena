package ca.frc6390.athena.runtime.geometry;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Objects;

/** Dependency-free point in a 2D coordinate system. */
public record Point2d(double x, double y) {
    public Point2d {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Point coordinates must be finite.");
        }
    }

    /** Creates a point from the translation of a pose snapshot. */
    public static Point2d from(PoseSnapshot pose) {
        Objects.requireNonNull(pose, "pose");
        return new Point2d(pose.xMeters(), pose.yMeters());
    }

    /** Returns the Euclidean distance to another point. */
    public double distance(Point2d other) {
        Objects.requireNonNull(other, "other");
        return Math.hypot(x - other.x, y - other.y);
    }
}
