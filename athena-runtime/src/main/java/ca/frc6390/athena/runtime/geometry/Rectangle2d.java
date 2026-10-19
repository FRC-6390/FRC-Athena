package ca.frc6390.athena.runtime.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

import static edu.wpi.first.units.Units.Meters;

/** Axis-aligned 2D rectangle. Stores its bounds as primitives in meters. */
public final class Rectangle2d implements Geometry2d {
    private final double minimumXMeters;
    private final double minimumYMeters;
    private final double maximumXMeters;
    private final double maximumYMeters;

    private Rectangle2d(double minimumX, double minimumY, double maximumX, double maximumY) {
        if (!Double.isFinite(minimumX) || !Double.isFinite(minimumY)
                || !Double.isFinite(maximumX) || !Double.isFinite(maximumY)) {
            throw new IllegalArgumentException("Rectangle coordinates must be finite.");
        }
        if (minimumX > maximumX || minimumY > maximumY) {
            throw new IllegalArgumentException("Rectangle minimum coordinates cannot exceed its maximums.");
        }
        this.minimumXMeters = minimumX;
        this.minimumYMeters = minimumY;
        this.maximumXMeters = maximumX;
        this.maximumYMeters = maximumY;
    }

    /** Creates a rectangle from its minimum and maximum coordinates. */
    public static Rectangle2d of(Distance minimumX, Distance minimumY, Distance maximumX, Distance maximumY) {
        return new Rectangle2d(
                Objects.requireNonNull(minimumX, "minimumX").in(Meters),
                Objects.requireNonNull(minimumY, "minimumY").in(Meters),
                Objects.requireNonNull(maximumX, "maximumX").in(Meters),
                Objects.requireNonNull(maximumY, "maximumY").in(Meters));
    }

    /** Creates field bounds whose origin is (0, 0). */
    public static Rectangle2d field(Distance length, Distance width) {
        double lengthMeters = Objects.requireNonNull(length, "length").in(Meters);
        double widthMeters = Objects.requireNonNull(width, "width").in(Meters);
        if (!(lengthMeters > 0.0) || !(widthMeters > 0.0)) {
            throw new IllegalArgumentException("Field dimensions must be positive.");
        }
        return new Rectangle2d(0.0, 0.0, lengthMeters, widthMeters);
    }

    public Distance minimumX() {
        return Meters.of(minimumXMeters);
    }

    public Distance minimumY() {
        return Meters.of(minimumYMeters);
    }

    public Distance maximumX() {
        return Meters.of(maximumXMeters);
    }

    public Distance maximumY() {
        return Meters.of(maximumYMeters);
    }

    /** Primitive minimum x in meters, for runtime geometry math. */
    double minimumXMeters() {
        return minimumXMeters;
    }

    /** Primitive minimum y in meters, for runtime geometry math. */
    double minimumYMeters() {
        return minimumYMeters;
    }

    /** Primitive maximum x in meters, for runtime geometry math. */
    double maximumXMeters() {
        return maximumXMeters;
    }

    /** Primitive maximum y in meters, for runtime geometry math. */
    double maximumYMeters() {
        return maximumYMeters;
    }

    @Override
    public double signedDistancePrimitive(Translation2d point) {
        double centerX = (minimumXMeters + maximumXMeters) * 0.5;
        double centerY = (minimumYMeters + maximumYMeters) * 0.5;
        double dx = Math.abs(point.getX() - centerX) - (maximumXMeters - minimumXMeters) * 0.5;
        double dy = Math.abs(point.getY() - centerY) - (maximumYMeters - minimumYMeters) * 0.5;
        double outside = Math.hypot(Math.max(dx, 0.0), Math.max(dy, 0.0));
        return outside + Math.min(Math.max(dx, dy), 0.0);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Rectangle2d other
                && Double.compare(minimumXMeters, other.minimumXMeters) == 0
                && Double.compare(minimumYMeters, other.minimumYMeters) == 0
                && Double.compare(maximumXMeters, other.maximumXMeters) == 0
                && Double.compare(maximumYMeters, other.maximumYMeters) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(minimumXMeters, minimumYMeters, maximumXMeters, maximumYMeters);
    }

    @Override
    public String toString() {
        return "Rectangle2d[minimumX=" + minimumXMeters + ", minimumY=" + minimumYMeters
                + ", maximumX=" + maximumXMeters + ", maximumY=" + maximumYMeters + "]";
    }
}
