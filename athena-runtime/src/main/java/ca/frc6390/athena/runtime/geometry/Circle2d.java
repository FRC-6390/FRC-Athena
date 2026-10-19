package ca.frc6390.athena.runtime.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

import static edu.wpi.first.units.Units.Meters;

/** Circle in a 2D coordinate system. Stores its radius as a primitive in meters. */
public final class Circle2d implements Geometry2d {
    private final Translation2d center;
    private final double radiusMeters;

    private Circle2d(Translation2d center, double radiusMeters) {
        this.center = Objects.requireNonNull(center, "center");
        if (!Double.isFinite(radiusMeters) || radiusMeters < 0.0) {
            throw new IllegalArgumentException("Circle radius must be finite and non-negative.");
        }
        this.radiusMeters = radiusMeters;
    }

    /** Creates a circle from its center and radius. */
    public static Circle2d of(Translation2d center, Distance radius) {
        return new Circle2d(center, Objects.requireNonNull(radius, "radius").in(Meters));
    }

    /** Creates a circle from its center coordinates and radius. */
    public static Circle2d of(Distance centerX, Distance centerY, Distance radius) {
        return of(
                new Translation2d(
                        Objects.requireNonNull(centerX, "centerX").in(Meters),
                        Objects.requireNonNull(centerY, "centerY").in(Meters)),
                radius);
    }

    public Translation2d center() {
        return center;
    }

    public Distance radius() {
        return Meters.of(radiusMeters);
    }

    @Override
    public double signedDistancePrimitive(Translation2d point) {
        return center.getDistance(point) - radiusMeters;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Circle2d other
                && center.equals(other.center)
                && Double.compare(radiusMeters, other.radiusMeters) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, radiusMeters);
    }

    @Override
    public String toString() {
        return "Circle2d[center=" + center + ", radius=" + radiusMeters + " m]";
    }
}
