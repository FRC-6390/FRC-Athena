package ca.frc6390.athena.runtime.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/** Simple polygon described by ordered vertices. */
public final class Polygon2d implements Geometry2d {
    private final List<Translation2d> vertices;

    private Polygon2d(List<Translation2d> vertices) {
        if (vertices.size() < 3) {
            throw new IllegalArgumentException("A polygon requires at least three vertices.");
        }
        this.vertices = List.copyOf(vertices);
        this.vertices.forEach(vertex -> Objects.requireNonNull(vertex, "vertex"));
    }

    /** Creates a polygon from ordered vertices. */
    public static Polygon2d of(Translation2d... vertices) {
        Objects.requireNonNull(vertices, "vertices");
        return new Polygon2d(Arrays.asList(vertices.clone()));
    }

    /** Returns this polygon's immutable ordered vertices. */
    public List<Translation2d> vertices() {
        return vertices;
    }

    @Override
    public double signedDistancePrimitive(Translation2d point) {
        Objects.requireNonNull(point, "point");
        boolean inside = false;
        double minimumDistance = Double.POSITIVE_INFINITY;
        for (int i = 0, previous = vertices.size() - 1; i < vertices.size(); previous = i++) {
            Translation2d a = vertices.get(previous);
            Translation2d b = vertices.get(i);
            minimumDistance = Math.min(minimumDistance, segmentDistance(point, a, b));
            if ((a.getY() > point.getY()) != (b.getY() > point.getY())
                    && point.getX() < (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX()) {
                inside = !inside;
            }
        }
        return inside ? -minimumDistance : minimumDistance;
    }

    private static double segmentDistance(Translation2d point, Translation2d start, Translation2d end) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared == 0.0) {
            return point.getDistance(start);
        }
        double amount = ((point.getX() - start.getX()) * dx + (point.getY() - start.getY()) * dy) / lengthSquared;
        double clamped = Math.max(0.0, Math.min(1.0, amount));
        return Math.hypot(point.getX() - (start.getX() + clamped * dx), point.getY() - (start.getY() + clamped * dy));
    }
}
