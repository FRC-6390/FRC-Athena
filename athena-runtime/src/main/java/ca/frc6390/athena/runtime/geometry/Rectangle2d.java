package ca.frc6390.athena.runtime.geometry;

/** Axis-aligned 2D rectangle. */
public record Rectangle2d(double minimumX, double minimumY, double maximumX, double maximumY)
        implements Geometry2d {
    public Rectangle2d {
        if (!Double.isFinite(minimumX) || !Double.isFinite(minimumY)
                || !Double.isFinite(maximumX) || !Double.isFinite(maximumY)) {
            throw new IllegalArgumentException("Rectangle coordinates must be finite.");
        }
        if (minimumX > maximumX || minimumY > maximumY) {
            throw new IllegalArgumentException("Rectangle minimum coordinates cannot exceed its maximums.");
        }
    }

    /** Creates a rectangle from its minimum and maximum coordinates. */
    public static Rectangle2d of(double minimumX, double minimumY, double maximumX, double maximumY) {
        return new Rectangle2d(minimumX, minimumY, maximumX, maximumY);
    }

    /** Creates field bounds whose origin is (0, 0). */
    public static Rectangle2d field(double length, double width) {
        if (!(length > 0.0) || !(width > 0.0)) {
            throw new IllegalArgumentException("Field dimensions must be positive.");
        }
        return new Rectangle2d(0.0, 0.0, length, width);
    }

    @Override
    public double signedDistance(Point2d point) {
        double centerX = (minimumX + maximumX) * 0.5;
        double centerY = (minimumY + maximumY) * 0.5;
        double dx = Math.abs(point.x() - centerX) - (maximumX - minimumX) * 0.5;
        double dy = Math.abs(point.y() - centerY) - (maximumY - minimumY) * 0.5;
        double outside = Math.hypot(Math.max(dx, 0.0), Math.max(dy, 0.0));
        return outside + Math.min(Math.max(dx, dy), 0.0);
    }
}
