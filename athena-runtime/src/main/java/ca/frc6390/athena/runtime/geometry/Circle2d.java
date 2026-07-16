package ca.frc6390.athena.runtime.geometry;

import java.util.Objects;

/** Circle in a 2D coordinate system. */
public record Circle2d(Point2d center, double radius) implements Geometry2d {
    public Circle2d {
        Objects.requireNonNull(center, "center");
        if (!Double.isFinite(radius) || radius < 0.0) {
            throw new IllegalArgumentException("Circle radius must be finite and non-negative.");
        }
    }

    /** Creates a circle from its center and radius. */
    public static Circle2d of(double centerX, double centerY, double radius) {
        return new Circle2d(new Point2d(centerX, centerY), radius);
    }

    @Override
    public double signedDistance(Point2d point) {
        return center.distance(point) - radius;
    }
}
