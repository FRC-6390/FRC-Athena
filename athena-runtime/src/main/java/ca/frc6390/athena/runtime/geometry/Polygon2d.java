package ca.frc6390.athena.runtime.geometry;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/** Simple polygon described by ordered vertices. */
public final class Polygon2d implements Geometry2d {
    private final List<Point2d> vertices;

    private Polygon2d(List<Point2d> vertices) {
        if (vertices.size() < 3) {
            throw new IllegalArgumentException("A polygon requires at least three vertices.");
        }
        this.vertices = List.copyOf(vertices);
        this.vertices.forEach(vertex -> Objects.requireNonNull(vertex, "vertex"));
    }

    /** Creates a polygon from ordered vertices. */
    public static Polygon2d of(Point2d... vertices) {
        Objects.requireNonNull(vertices, "vertices");
        return new Polygon2d(Arrays.asList(vertices.clone()));
    }

    /** Returns this polygon's immutable ordered vertices. */
    public List<Point2d> vertices() {
        return vertices;
    }

    @Override
    public double signedDistance(Point2d point) {
        Objects.requireNonNull(point, "point");
        boolean inside = false;
        double minimumDistance = Double.POSITIVE_INFINITY;
        for (int i = 0, previous = vertices.size() - 1; i < vertices.size(); previous = i++) {
            Point2d a = vertices.get(previous);
            Point2d b = vertices.get(i);
            minimumDistance = Math.min(minimumDistance, segmentDistance(point, a, b));
            if ((a.y() > point.y()) != (b.y() > point.y())
                    && point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x()) {
                inside = !inside;
            }
        }
        return inside ? -minimumDistance : minimumDistance;
    }

    private static double segmentDistance(Point2d point, Point2d start, Point2d end) {
        double dx = end.x() - start.x();
        double dy = end.y() - start.y();
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared == 0.0) {
            return point.distance(start);
        }
        double amount = ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / lengthSquared;
        double clamped = Math.max(0.0, Math.min(1.0, amount));
        return Math.hypot(point.x() - (start.x() + clamped * dx), point.y() - (start.y() + clamped * dy));
    }
}
