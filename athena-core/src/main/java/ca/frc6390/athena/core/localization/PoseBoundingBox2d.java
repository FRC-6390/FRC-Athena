package ca.frc6390.athena.core.localization;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Axis-aligned 2D bounding box used for localization checks.
 *
 * <p>All units are meters in the field coordinate frame.
 */
public record PoseBoundingBox2d(double minX, double minY, double maxX, double maxY) {

    public PoseBoundingBox2d {
        if (!Double.isFinite(minX)
                || !Double.isFinite(minY)
                || !Double.isFinite(maxX)
                || !Double.isFinite(maxY)) {
            throw new IllegalArgumentException("Bounding box coordinates must be finite.");
        }

        if (maxX < minX) {
            double swap = minX;
            minX = maxX;
            maxX = swap;
        }
        if (maxY < minY) {
            double swap = minY;
            minY = maxY;
            maxY = swap;
        }
    }

    public static PoseBoundingBox2d fromCorners(Translation2d cornerA, Translation2d cornerB) {
        Objects.requireNonNull(cornerA, "cornerA");
        Objects.requireNonNull(cornerB, "cornerB");
        return new PoseBoundingBox2d(cornerA.getX(), cornerA.getY(), cornerB.getX(), cornerB.getY());
    }

    public static PoseBoundingBox2d fromCorners(Pose2d cornerA, Pose2d cornerB) {
        Objects.requireNonNull(cornerA, "cornerA");
        Objects.requireNonNull(cornerB, "cornerB");
        return fromCorners(cornerA.getTranslation(), cornerB.getTranslation());
    }

    /**
     * Returns a new bounding box with corners replaced by the provided translations.
     */
    public PoseBoundingBox2d updateCorners(Translation2d cornerA, Translation2d cornerB) {
        return fromCorners(cornerA, cornerB);
    }

    /**
     * Returns a new bounding box with corners replaced by the provided poses.
     */
    public PoseBoundingBox2d updateCorners(Pose2d cornerA, Pose2d cornerB) {
        return fromCorners(cornerA, cornerB);
    }

    /**
     * Returns a new bounding box with corners replaced by the provided coordinates.
     */
    public PoseBoundingBox2d updateCorners(double minX, double minY, double maxX, double maxY) {
        return new PoseBoundingBox2d(minX, minY, maxX, maxY);
    }

    public boolean contains(Translation2d point) {
        if (point == null) {
            return false;
        }
        return contains(point.getX(), point.getY());
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    public boolean contains(Pose2d pose) {
        return pose != null && contains(pose.getTranslation());
    }
}
