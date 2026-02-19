package ca.frc6390.athena.core.localization;

import java.util.Objects;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Axis-aligned 2D bounding box used for localization checks.
 *
 * <p>All units are meters in the field coordinate frame.
 */
public record PoseBoundingBox2d(double minX, double minY, double maxX, double maxY) {
    /**
     * Default field dimensions (meters) used by convenience mirror helpers.
     */
    public static final double DEFAULT_FIELD_LENGTH_METERS = 16.54;
    public static final double DEFAULT_FIELD_WIDTH_METERS = 8.21;

    /**
     * Built-in pivot anchors for box rotation.
     */
    public enum RotationPivot {
        ORIGIN,
        CENTER,
        LEFT,
        RIGHT,
        TOP,
        BOTTOM,
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT
    }

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

    /**
     * Rotates this box around the provided origin and returns the enclosing axis-aligned box of the
     * rotated corners.
     */
    public PoseBoundingBox2d rotate(Rotation2d rotation, Translation2d origin) {
        Objects.requireNonNull(rotation, "rotation");
        Objects.requireNonNull(origin, "origin");
        return transformByPointOperator(point -> rotatePoint(point, rotation, origin));
    }

    /**
     * Rotates this box around a built-in pivot and returns the enclosing axis-aligned box of the
     * rotated corners.
     */
    public PoseBoundingBox2d rotate(Rotation2d rotation, RotationPivot pivot) {
        Objects.requireNonNull(rotation, "rotation");
        Objects.requireNonNull(pivot, "pivot");
        return rotate(rotation, resolvePivot(pivot));
    }

    /**
     * Rotates this box around the global origin (0,0).
     */
    public PoseBoundingBox2d rotateAroundOrigin(Rotation2d rotation) {
        return rotate(rotation, RotationPivot.ORIGIN);
    }

    /**
     * Mirrors this box across a vertical axis line {@code x = axisX}.
     */
    public PoseBoundingBox2d mirrorVertical(double axisX) {
        double resolvedAxisX = requireFiniteAxis("axisX", axisX);
        return new PoseBoundingBox2d(
                reflect(maxX, resolvedAxisX),
                minY,
                reflect(minX, resolvedAxisX),
                maxY);
    }

    /**
     * Mirrors this box across a horizontal axis line {@code y = axisY}.
     */
    public PoseBoundingBox2d mirrorHorizontal(double axisY) {
        double resolvedAxisY = requireFiniteAxis("axisY", axisY);
        return new PoseBoundingBox2d(
                minX,
                reflect(maxY, resolvedAxisY),
                maxX,
                reflect(minY, resolvedAxisY));
    }

    /**
     * Mirrors this box across both axis lines {@code x = axisX} and {@code y = axisY}.
     */
    public PoseBoundingBox2d mirror(double axisX, double axisY) {
        return mirrorVertical(axisX).mirrorHorizontal(axisY);
    }

    /**
     * Mirrors this box across both axis lines passing through {@code axisPoint}.
     */
    public PoseBoundingBox2d mirror(Translation2d axisPoint) {
        Objects.requireNonNull(axisPoint, "axisPoint");
        return mirror(axisPoint.getX(), axisPoint.getY());
    }

    /**
     * Mirrors this box from one alliance side to the other using default field dimensions.
     *
     * <p>This convenience overload assumes the current box is authored for the Blue side.</p>
     */
    public PoseBoundingBox2d mirrorToAlliance(DriverStation.Alliance alliance) {
        return mirrorToAlliance(alliance, DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors this box from one alliance side to the other based on the requested alliance.
     *
     * <p>This convenience overload assumes the current box is authored for the Blue side.</p>
     */
    public PoseBoundingBox2d mirrorToAlliance(
            DriverStation.Alliance alliance,
            double fieldLength,
            double fieldWidth) {
        return mirrorAlliance(DriverStation.Alliance.Blue, alliance, fieldLength, fieldWidth);
    }

    /**
     * Mirrors this box from a source alliance frame to a target alliance frame using
     * default field dimensions.
     */
    public PoseBoundingBox2d mirrorAlliance(
            DriverStation.Alliance sourceAlliance,
            DriverStation.Alliance targetAlliance) {
        return mirrorAlliance(sourceAlliance, targetAlliance, DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors this box from a source alliance frame to a target alliance frame.
     */
    public PoseBoundingBox2d mirrorAlliance(
            DriverStation.Alliance sourceAlliance,
            DriverStation.Alliance targetAlliance,
            double fieldLength,
            double fieldWidth) {
        Objects.requireNonNull(sourceAlliance, "sourceAlliance");
        Objects.requireNonNull(targetAlliance, "targetAlliance");
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        if (sourceAlliance == targetAlliance) {
            return this;
        }
        return mirror(resolvedFieldLength * 0.5, resolvedFieldWidth * 0.5);
    }

    /**
     * Mirrors this box across the field horizontal midline (top-to-bottom).
     */
    public PoseBoundingBox2d mirrorTopBottom(double fieldWidth) {
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return mirrorHorizontal(resolvedFieldWidth * 0.5);
    }

    /**
     * Mirrors this box across the default-field horizontal midline (top-to-bottom).
     */
    public PoseBoundingBox2d mirrorTopBottom() {
        return mirrorTopBottom(DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors this box across the field vertical midline (left-to-right).
     */
    public PoseBoundingBox2d mirrorLeftRight(double fieldLength) {
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        return mirrorVertical(resolvedFieldLength * 0.5);
    }

    /**
     * Mirrors this box across the default-field vertical midline (left-to-right).
     */
    public PoseBoundingBox2d mirrorLeftRight() {
        return mirrorLeftRight(DEFAULT_FIELD_LENGTH_METERS);
    }

    /**
     * Mirrors this box across the main diagonal from (0,0) to (fieldLength, fieldWidth).
     *
     * <p>Diagonal reflection can rotate an axis-aligned box. This method returns the enclosing
     * axis-aligned box of the mirrored corners.</p>
     */
    public PoseBoundingBox2d mirrorDiagonalMain(double fieldLength, double fieldWidth) {
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return transformByPointOperator(
                point -> mirrorPointDiagonalMain(point, resolvedFieldLength, resolvedFieldWidth));
    }

    /**
     * Mirrors this box across the default-field main diagonal.
     */
    public PoseBoundingBox2d mirrorDiagonalMain() {
        return mirrorDiagonalMain(DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors this box across the anti-diagonal from (0, fieldWidth) to (fieldLength, 0).
     *
     * <p>Diagonal reflection can rotate an axis-aligned box. This method returns the enclosing
     * axis-aligned box of the mirrored corners.</p>
     */
    public PoseBoundingBox2d mirrorDiagonalAnti(double fieldLength, double fieldWidth) {
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return transformByPointOperator(
                point -> mirrorPointDiagonalAnti(point, resolvedFieldLength, resolvedFieldWidth));
    }

    /**
     * Mirrors this box across the default-field anti-diagonal.
     */
    public PoseBoundingBox2d mirrorDiagonalAnti() {
        return mirrorDiagonalAnti(DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors a point across a vertical axis line {@code x = axisX}.
     */
    public static Translation2d mirrorPointVertical(Translation2d point, double axisX) {
        Objects.requireNonNull(point, "point");
        double resolvedAxisX = requireFiniteAxis("axisX", axisX);
        return new Translation2d(reflect(point.getX(), resolvedAxisX), point.getY());
    }

    /**
     * Mirrors a point across a horizontal axis line {@code y = axisY}.
     */
    public static Translation2d mirrorPointHorizontal(Translation2d point, double axisY) {
        Objects.requireNonNull(point, "point");
        double resolvedAxisY = requireFiniteAxis("axisY", axisY);
        return new Translation2d(point.getX(), reflect(point.getY(), resolvedAxisY));
    }

    /**
     * Mirrors a point across both axis lines {@code x = axisX} and {@code y = axisY}.
     */
    public static Translation2d mirrorPoint(Translation2d point, double axisX, double axisY) {
        return mirrorPointHorizontal(mirrorPointVertical(point, axisX), axisY);
    }

    /**
     * Mirrors a point across both axis lines passing through {@code axisPoint}.
     */
    public static Translation2d mirrorPoint(Translation2d point, Translation2d axisPoint) {
        Objects.requireNonNull(axisPoint, "axisPoint");
        return mirrorPoint(point, axisPoint.getX(), axisPoint.getY());
    }

    /**
     * Mirrors a point from one alliance side to the other based on the requested alliance.
     *
     * <p>This convenience overload assumes the current point is authored for the Blue side.</p>
     */
    public static Translation2d mirrorPointToAlliance(
            Translation2d point,
            DriverStation.Alliance alliance,
            double fieldLength,
            double fieldWidth) {
        return mirrorPointAlliance(point, DriverStation.Alliance.Blue, alliance, fieldLength, fieldWidth);
    }

    /**
     * Mirrors a point from one alliance side to the other using default field dimensions.
     */
    public static Translation2d mirrorPointToAlliance(
            Translation2d point,
            DriverStation.Alliance alliance) {
        return mirrorPointToAlliance(point, alliance, DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors a point from a source alliance frame to a target alliance frame using
     * default field dimensions.
     */
    public static Translation2d mirrorPointAlliance(
            Translation2d point,
            DriverStation.Alliance sourceAlliance,
            DriverStation.Alliance targetAlliance) {
        return mirrorPointAlliance(
                point,
                sourceAlliance,
                targetAlliance,
                DEFAULT_FIELD_LENGTH_METERS,
                DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors a point from a source alliance frame to a target alliance frame.
     */
    public static Translation2d mirrorPointAlliance(
            Translation2d point,
            DriverStation.Alliance sourceAlliance,
            DriverStation.Alliance targetAlliance,
            double fieldLength,
            double fieldWidth) {
        Objects.requireNonNull(point, "point");
        Objects.requireNonNull(sourceAlliance, "sourceAlliance");
        Objects.requireNonNull(targetAlliance, "targetAlliance");
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        if (sourceAlliance == targetAlliance) {
            return point;
        }
        return mirrorPoint(point, resolvedFieldLength * 0.5, resolvedFieldWidth * 0.5);
    }

    /**
     * Mirrors a point across the field horizontal midline (top-to-bottom).
     */
    public static Translation2d mirrorPointTopBottom(Translation2d point, double fieldWidth) {
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return mirrorPointHorizontal(point, resolvedFieldWidth * 0.5);
    }

    /**
     * Mirrors a point across the default-field horizontal midline (top-to-bottom).
     */
    public static Translation2d mirrorPointTopBottom(Translation2d point) {
        return mirrorPointTopBottom(point, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors a point across the field vertical midline (left-to-right).
     */
    public static Translation2d mirrorPointLeftRight(Translation2d point, double fieldLength) {
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        return mirrorPointVertical(point, resolvedFieldLength * 0.5);
    }

    /**
     * Mirrors a point across the default-field vertical midline (left-to-right).
     */
    public static Translation2d mirrorPointLeftRight(Translation2d point) {
        return mirrorPointLeftRight(point, DEFAULT_FIELD_LENGTH_METERS);
    }

    /**
     * Mirrors a point across the main diagonal from (0,0) to (fieldLength, fieldWidth).
     */
    public static Translation2d mirrorPointDiagonalMain(
            Translation2d point,
            double fieldLength,
            double fieldWidth) {
        Objects.requireNonNull(point, "point");
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return reflectPointAcrossLine(point, new Translation2d(0.0, 0.0), resolvedFieldLength, resolvedFieldWidth);
    }

    /**
     * Mirrors a point across the default-field main diagonal.
     */
    public static Translation2d mirrorPointDiagonalMain(Translation2d point) {
        return mirrorPointDiagonalMain(point, DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Mirrors a point across the anti-diagonal from (0, fieldWidth) to (fieldLength, 0).
     */
    public static Translation2d mirrorPointDiagonalAnti(
            Translation2d point,
            double fieldLength,
            double fieldWidth) {
        Objects.requireNonNull(point, "point");
        double resolvedFieldLength = requirePositiveFinite("fieldLength", fieldLength);
        double resolvedFieldWidth = requirePositiveFinite("fieldWidth", fieldWidth);
        return reflectPointAcrossLine(
                point,
                new Translation2d(0.0, resolvedFieldWidth),
                resolvedFieldLength,
                -resolvedFieldWidth);
    }

    /**
     * Mirrors a point across the default-field anti-diagonal.
     */
    public static Translation2d mirrorPointDiagonalAnti(Translation2d point) {
        return mirrorPointDiagonalAnti(point, DEFAULT_FIELD_LENGTH_METERS, DEFAULT_FIELD_WIDTH_METERS);
    }

    /**
     * Rotates a point around the provided origin.
     */
    public static Translation2d rotatePoint(Translation2d point, Rotation2d rotation, Translation2d origin) {
        Objects.requireNonNull(point, "point");
        Objects.requireNonNull(rotation, "rotation");
        Objects.requireNonNull(origin, "origin");
        double cos = rotation.getCos();
        double sin = rotation.getSin();
        double dx = point.getX() - origin.getX();
        double dy = point.getY() - origin.getY();
        double rotatedX = origin.getX() + dx * cos - dy * sin;
        double rotatedY = origin.getY() + dx * sin + dy * cos;
        return new Translation2d(rotatedX, rotatedY);
    }

    /**
     * Rotates a point around the global origin (0,0).
     */
    public static Translation2d rotatePoint(Translation2d point, Rotation2d rotation) {
        return rotatePoint(point, rotation, new Translation2d());
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

    private static double requireFiniteAxis(String axisName, double axisValue) {
        if (!Double.isFinite(axisValue)) {
            throw new IllegalArgumentException(axisName + " must be finite.");
        }
        return axisValue;
    }

    private static double requirePositiveFinite(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and > 0.");
        }
        return value;
    }

    private static double reflect(double value, double axis) {
        return (2.0 * axis) - value;
    }

    private PoseBoundingBox2d transformByPointOperator(UnaryOperator<Translation2d> transform) {
        Translation2d c1 = transform.apply(new Translation2d(minX, minY));
        Translation2d c2 = transform.apply(new Translation2d(minX, maxY));
        Translation2d c3 = transform.apply(new Translation2d(maxX, minY));
        Translation2d c4 = transform.apply(new Translation2d(maxX, maxY));
        double mirroredMinX = Math.min(Math.min(c1.getX(), c2.getX()), Math.min(c3.getX(), c4.getX()));
        double mirroredMaxX = Math.max(Math.max(c1.getX(), c2.getX()), Math.max(c3.getX(), c4.getX()));
        double mirroredMinY = Math.min(Math.min(c1.getY(), c2.getY()), Math.min(c3.getY(), c4.getY()));
        double mirroredMaxY = Math.max(Math.max(c1.getY(), c2.getY()), Math.max(c3.getY(), c4.getY()));
        return new PoseBoundingBox2d(mirroredMinX, mirroredMinY, mirroredMaxX, mirroredMaxY);
    }

    private Translation2d resolvePivot(RotationPivot pivot) {
        double centerX = (minX + maxX) * 0.5;
        double centerY = (minY + maxY) * 0.5;
        return switch (pivot) {
            case ORIGIN -> new Translation2d();
            case CENTER -> new Translation2d(centerX, centerY);
            case LEFT -> new Translation2d(minX, centerY);
            case RIGHT -> new Translation2d(maxX, centerY);
            case TOP -> new Translation2d(centerX, maxY);
            case BOTTOM -> new Translation2d(centerX, minY);
            case TOP_LEFT -> new Translation2d(minX, maxY);
            case TOP_RIGHT -> new Translation2d(maxX, maxY);
            case BOTTOM_LEFT -> new Translation2d(minX, minY);
            case BOTTOM_RIGHT -> new Translation2d(maxX, minY);
        };
    }

    private static Translation2d reflectPointAcrossLine(
            Translation2d point,
            Translation2d linePoint,
            double directionX,
            double directionY) {
        Objects.requireNonNull(point, "point");
        Objects.requireNonNull(linePoint, "linePoint");
        if (!Double.isFinite(directionX) || !Double.isFinite(directionY)) {
            throw new IllegalArgumentException("line direction must be finite.");
        }
        double norm = Math.hypot(directionX, directionY);
        if (norm <= 1e-9) {
            throw new IllegalArgumentException("line direction must be non-zero.");
        }
        double ux = directionX / norm;
        double uy = directionY / norm;
        double relX = point.getX() - linePoint.getX();
        double relY = point.getY() - linePoint.getY();
        double dot = relX * ux + relY * uy;
        double projX = dot * ux;
        double projY = dot * uy;
        double reflectedX = linePoint.getX() + (2.0 * projX - relX);
        double reflectedY = linePoint.getY() + (2.0 * projY - relY);
        return new Translation2d(reflectedX, reflectedY);
    }
}
